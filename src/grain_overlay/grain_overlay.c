/**
 * grain_overlay - blend a scanned film-grain plate into still photos (DNG output)
 *
 * Targets: Canon 550D (DIGIC 4, raw buffer source) and 750D (DIGIC 6, CR2
 * source). Other DIGIC 4/5 bodies with CONFIG_RAW_PHOTO and other CR2 bodies
 * (DIGIC 6/7) should work too, but were not designed for.
 *
 * USAGE
 *   1. Copy the plate to the card as  ML/GRAIN.GRN
 *   2. Canon menu: picture quality RAW or RAW+JPEG
 *      (raw buffer source only: image review ON, 2s or more)
 *   3. ML menu -> Shoot -> Grain Overlay: ON
 *   4. Shoot normally. After each picture, DCIM/xxxCANON/GRN_nnnn.DNG is written
 *      (nnnn = Canon file number, so it pairs with IMG_nnnn.CR2).
 *
 * SOURCE (menu; Auto = raw buffer on DIGIC 4/5, CR2 file on DIGIC 6/7)
 *   Raw buffer: ML's photo raw access during image review (below). DIGIC 6+
 *     ports have none (no CONFIG_RAW_PHOTO, no edmac_memcpy).
 *   CR2 file: a background task waits for Canon's IMG_nnnn.CR2, decodes its
 *     lossless JPEG stream and writes the DNG in strips (see grain_cr2.h).
 *     Needs ~0.5 MB, no image review; takes several seconds per picture.
 *   Optional core symbols are imported WEAK: a hard import of a symbol the
 *   camera lacks makes ML's linker fail for ALL modules.
 *
 * HOW IT WORKS (raw buffer source)
 *   There is no "raw capture hook" in ML. The photo raw buffer is only reachable
 *   while Canon's image review (QR) is on screen. When PROP_GUI_STATE switches to
 *   QR, a worker task:
 *     a) calls raw_update_params() and snapshots raw_info,
 *     b) copies the packed 14-bit frame (~31 MB) into its own buffer (EDMAC),
 *        and verifies QR is still active (otherwise the copy may be stale),
 *   -- from here on, Canon's buffer is no longer needed --
 *     c) blends row by row: unpack row -> blend with plate row -> repack in place,
 *        streaming the plate from the card in strips (plate is never fully in RAM),
 *     d) writes the result with ML's save_dng().
 *   Peak memory: one packed frame (~31 MB) + ~1 MB strip/row buffers.
 *
 * PLATE FORMAT (.grn, little endian)
 *   0  "GRN1"      4  version=1     8  width     12 height
 *   16 black_level 20 white_level   24 width*height uint16 (0..16383), row-major
 *   width/height must equal the raw ACTIVE AREA reported by the camera. The menu
 *   shows the required size after the first picture (550D: expect 5202x3464,
 *   not 5184x3456 - ML's active area includes a few more pixels than JPEG size;
 *   CR2 source uses the makernote crop, 750D: 6000x4000).
 *
 * BLEND (Strength s, plate b normalized 0..1, photo a normalized 0..1, linear)
 *   Screen : 1-(1-a)(1-s*b)       Lighten: max(a, s*b)
 *   Overlay: a + s*(overlay(a,b) - a)   (plate centered at 50% gray)
 *   Strength 0 = bit-exact passthrough (used for the round-trip test).
 *
 * Save original = OFF deletes Canon's CR2/JPG after the DNG was written
 * successfully (experimental; never deletes if anything failed).
 */

#include <dryos.h>
#include <module.h>
#include <menu.h>
#include <config.h>
#include <property.h>
#include <raw.h>
#include <lens.h>
#include <bmp.h>
#include <shoot.h>
#include <fileprefix.h>
#include <edmac-memcpy.h>
#include <chdk-dng.h>
#include <propvalues.h>

#include "grain_core.h"
#include "grain_cr2.h"

#define PLATE_PATH      "ML/GRAIN.GRN"
#define LOG_PATH        "ML/LOGS/GRAIN.LOG"
#define STRIP_ROWS      32

/* CR2 source buffers */
#define CR2_STRIP_ROWS  16
#define CR2_CHUNK       (64 * 1024)

/* small DMA-capable buffers for FIO; not fio_malloc, which prefers shoot memory */
#define dma_malloc(len) __mem_malloc(len, MEM_DMA, __FILE__, __LINE__)

/* not exported on every camera (e.g. DIGIC 6 has no edmac_memcpy); ret_0 when missing */
extern WEAK_FUNC(ret_0) void* edmac_memcpy(void* dest, void* srce, size_t n);
extern WEAK_FUNC(ret_0) int get_digic_version(void);
/* cam independent directory entries (newer cores only; both or neither) */
extern WEAK_FUNC(ret_0) struct fio_file * alloc_fio_file(void);
extern WEAK_FUNC(ret_0) struct file_info convert_fio_file_info(struct fio_file *);

static CONFIG_INT("grain.enabled",  grain_enabled,  0);
static CONFIG_INT("grain.strength", grain_strength, 50);
static CONFIG_INT("grain.mode",     grain_mode,     GRN_BLEND_SCREEN);
static CONFIG_INT("grain.keep",     grain_keep_orig, 1);
static CONFIG_INT("grain.source",   grain_source,   0);
static CONFIG_INT("grain.debug",    grain_debug,    0);

#define SOURCE_AUTO     0
#define SOURCE_RAWBUF   1
#define SOURCE_CR2      2

/* 0 = old core without get_digic_version (all of those are DIGIC 4/5) */
static int use_cr2(void)
{
    if (grain_source != SOURCE_AUTO) return grain_source == SOURCE_CR2;
    return get_digic_version() >= 6;
}

/* raw buffer: ML photo raw access exists for DIGIC 4/5 only; CR2: DIGIC 8+ writes CR3 */
static int source_supported(void)
{
    int digic = get_digic_version();
    return use_cr2() ? digic < 8 : digic <= 5;
}

/* debug log: console + ML/LOGS/GRAIN.LOG (rewritten after each picture) */
static char log_buf[4096];
static int log_len = 0;

#define LOG(fmt, ...) do { if (grain_debug) { \
    printf("[grain] " fmt "\n", ## __VA_ARGS__); \
    if (log_len < (int) sizeof(log_buf) - 1) { \
        log_len += snprintf(log_buf + log_len, sizeof(log_buf) - log_len, fmt "\n", ## __VA_ARGS__); \
        if (log_len > (int) sizeof(log_buf) - 1) log_len = sizeof(log_buf) - 1; } \
    } } while (0)

static void log_flush(void)
{
    if (!grain_debug || !log_len) return;
    FILE * f = FIO_CreateFile(LOG_PATH);
    if (!f) return;
    FIO_WriteFile(f, log_buf, log_len);
    FIO_CloseFile(f);
}

/* plate header info (validated at load; payload is streamed per shot) */
static int plate_ok = 0;
static int plate_w, plate_h, plate_black, plate_white;
static char plate_err[48] = "not loaded";

/* last known raw active area (for the menu), 0 = unknown yet */
static int cam_active_w = 0, cam_active_h = 0;

static volatile int job_busy = 0;
static int last_file_number = -1;
static char last_result[48] = "";

/* ------------------------------------------------------------------ */
/* plate                                                               */
/* ------------------------------------------------------------------ */

static void plate_load_header(void)
{
    plate_ok = 0;

    uint32_t size = 0;
    if (FIO_GetFileSize(PLATE_PATH, &size) != 0)
    {
        snprintf(plate_err, sizeof(plate_err), "%s missing", PLATE_PATH);
        return;
    }

    FILE * f = FIO_OpenFile(PLATE_PATH, O_RDONLY | O_SYNC);
    if (!f)
    {
        snprintf(plate_err, sizeof(plate_err), "cannot open plate");
        return;
    }

    uint8_t * hdr = fio_malloc(GRN_HEADER_SIZE);
    int r = hdr ? FIO_ReadFile(f, hdr, GRN_HEADER_SIZE) : -1;
    FIO_CloseFile(f);

    if (r != GRN_HEADER_SIZE)
    {
        snprintf(plate_err, sizeof(plate_err), "header read error");
        if (hdr) free(hdr);
        return;
    }

    int err = grn_parse_header(hdr, size, &plate_w, &plate_h, &plate_black, &plate_white);
    free(hdr);

    switch (err)
    {
        case 0:  plate_ok = 1; snprintf(plate_err, sizeof(plate_err), "OK"); break;
        case -1: snprintf(plate_err, sizeof(plate_err), "bad magic (not GRN1)"); break;
        case -2: snprintf(plate_err, sizeof(plate_err), "unsupported version"); break;
        case -3: snprintf(plate_err, sizeof(plate_err), "bad dimensions"); break;
        case -4: snprintf(plate_err, sizeof(plate_err), "bad black/white level"); break;
        default: snprintf(plate_err, sizeof(plate_err), "file size mismatch"); break;
    }
    LOG("plate: %s (%dx%d, %d..%d)", plate_err, plate_w, plate_h, plate_black, plate_white);
}

/* ------------------------------------------------------------------ */
/* worker                                                              */
/* ------------------------------------------------------------------ */

static void set_result(const char * msg)
{
    snprintf(last_result, sizeof(last_result), "%s", msg);
    LOG("%s", msg);
}

static void delete_originals(int file_number)
{
    char fn[64];
    const char * ext[] = { "CR2", "JPG" };
    for (int i = 0; i < 2; i++)
    {
        snprintf(fn, sizeof(fn), "%s/%s%04d.%s", get_dcim_dir(), get_file_prefix(), file_number, ext[i]);

        /* Canon may still be flushing the file: wait until it exists and its size is stable */
        uint32_t s0 = 0, s1 = 0;
        int stable = 0;
        for (int t = 0; t < 100 && !stable; t++)   /* max ~10 s */
        {
            if (FIO_GetFileSize(fn, &s0) == 0)
            {
                msleep(100);
                if (FIO_GetFileSize(fn, &s1) == 0 && s0 == s1 && s0 > 0) stable = 1;
            }
            else if (i == 1) break;   /* no JPG in RAW-only mode: fine */
            else msleep(100);
        }
        if (stable)
        {
            FIO_RemoveFile(fn);
            LOG("deleted %s", fn);
        }
    }
}

static void grain_task(void * unused)
{
    void * frame = 0;
    FILE * pf = 0;
    uint16_t * row = 0;
    uint8_t * packed_row = 0;
    uint16_t * plate_row = 0;
    uint16_t * plate_strip = 0;   /* DMA memory (FIO reads) */

    int file_number = get_shooting_card()->file_number;
    if (file_number == last_file_number)
    {
        /* QR -> QR zoom -> QR fires again for the same picture */
        goto done;
    }

    /* 1. wait for raw data of this picture (gui_state may lag behind the property) */
    int ok = 0;
    for (int i = 0; i < 20 && !ok; i++)
    {
        if (QR_MODE) ok = raw_update_params();
        if (!ok) msleep(50);
    }
    if (!ok)
    {
        set_result("no raw data (RAW + review on?)");
        goto done;
    }

    struct raw_info ri = raw_info;   /* snapshot: raw_info is global and shared */

    int aw = ri.active_area.x2 - ri.active_area.x1;
    int ah = ri.active_area.y2 - ri.active_area.y1;
    cam_active_w = aw;
    cam_active_h = ah;
    LOG("raw %dx%d pitch %d, active %d,%d-%d,%d, black %d white %d",
        ri.width, ri.height, ri.pitch, ri.active_area.x1, ri.active_area.y1,
        ri.active_area.x2, ri.active_area.y2, ri.black_level, ri.white_level);

    if (ri.bits_per_pixel != 14 || (ri.width & 7) || ri.pitch != ri.width * 14 / 8)
    {
        set_result("unexpected raw format");
        goto done;
    }

    const int do_blend = grain_strength > 0;
    struct grn_blend_params bp = { 0 };

    if (do_blend)
    {
        if (!plate_ok)
        {
            set_result("plate not loaded");
            goto done;
        }
        if (plate_w != aw || plate_h != ah)
        {
            char msg[48];
            snprintf(msg, sizeof(msg), "plate %dx%d, need %dx%d", plate_w, plate_h, aw, ah);
            set_result(msg);
            NotifyBox(3000, "Grain: plate is %dx%d,\nneeds %dx%d", plate_w, plate_h, aw, ah);
            goto done;
        }
        if (grn_blend_setup(&bp, grain_mode, grain_strength, ri.black_level, ri.white_level,
                            plate_black, plate_white) != 0)
        {
            set_result("bad levels");
            goto done;
        }
    }

    /* 2. copy Canon's frame while QR is still active */
    frame = fio_malloc(ri.frame_size);
    if (!frame)
    {
        set_result("not enough memory");
        NotifyBox(2000, "Grain: not enough memory");
        goto done;
    }

    int t0 = get_ms_clock();
    if (edmac_memcpy(frame, ri.buffer, ri.frame_size) != frame)
        memcpy(frame, UNCACHEABLE(ri.buffer), ri.frame_size);
    int t_copy = get_ms_clock() - t0;

    if (!QR_MODE)
    {
        /* review ended during the copy: buffer may already hold a newer frame */
        set_result("review ended too early");
        goto done;
    }

    ri.buffer = frame;
    last_file_number = file_number;
    LOG("copied %d bytes in %d ms", ri.frame_size, t_copy);

    /* 3. blend, row by row, streaming the plate */
    int t_blend = 0;
    if (do_blend)
    {
        t0 = get_ms_clock();
        row         = malloc(ri.width * sizeof(uint16_t));
        packed_row  = malloc(ri.pitch);
        plate_row   = malloc(plate_w * sizeof(uint16_t));
        plate_strip = fio_malloc(plate_w * sizeof(uint16_t) * STRIP_ROWS);
        pf          = FIO_OpenFile(PLATE_PATH, O_RDONLY | O_SYNC);

        if (!row || !packed_row || !plate_row || !plate_strip || !pf)
        {
            set_result("blend setup failed");
            goto done;
        }
        if (FIO_SeekSkipFile(pf, GRN_HEADER_SIZE, SEEK_SET) != GRN_HEADER_SIZE)
        {
            set_result("plate seek failed");
            goto done;
        }

        const int x1 = ri.active_area.x1;
        const int y1 = ri.active_area.y1;
        const int plate_row_bytes = plate_w * sizeof(uint16_t);

        for (int py = 0; py < plate_h; py += STRIP_ROWS)
        {
            int n = MIN(STRIP_ROWS, plate_h - py);
            if (FIO_ReadFile(pf, plate_strip, n * plate_row_bytes) != n * plate_row_bytes)
            {
                set_result("plate read error");
                goto done;
            }

            for (int j = 0; j < n; j++)
            {
                uint8_t * dst = (uint8_t *) frame + (y1 + py + j) * ri.pitch;

                /* uncached big buffer <-> cached small row buffers */
                memcpy(packed_row, dst, ri.pitch);
                memcpy(plate_row, (uint8_t *) plate_strip + j * plate_row_bytes, plate_row_bytes);

                grn_unpack_row(packed_row, row, ri.width);
                grn_blend_span(&bp, row, x1, plate_row, plate_w);
                grn_pack_row(row, packed_row, ri.width);

                memcpy(dst, packed_row, ri.pitch);
            }

            msleep(1);   /* let other tasks breathe */
        }
        t_blend = get_ms_clock() - t0;
        LOG("blended in %d ms", t_blend);
    }

    /* 4. write DNG (frame is only written through its uncacheable alias, so no cache flush needed) */
    char fn[64];
    snprintf(fn, sizeof(fn), "%s/GRN_%04d.DNG", get_dcim_dir(), file_number);

    int iso = lens_info.iso ? (int) lens_info.iso : raw2iso(lens_info.raw_iso_auto);
    dng_set_iso(iso);
    if (lens_info.raw_shutter)
    {
        /* exposure time in microseconds (raw2shutterf returns seconds) */
        int us = (int)(raw2shutterf(lens_info.raw_shutter) * 1000000.0f + 0.5f);
        if (us > 0) dng_set_shutter(us, 1000000);
    }
    dng_set_aperture(lens_info.aperture, 10);
    dng_set_focal(lens_info.focal_len, 1);

    t0 = get_ms_clock();
    int saved = save_dng(fn, &ri);   /* note: byte-swaps the buffer in place */
    int t_save = get_ms_clock() - t0;

    if (!saved)
    {
        set_result("DNG write failed");
        NotifyBox(2000, "Grain: DNG write failed");
        goto done;
    }

    {
        char msg[48];
        snprintf(msg, sizeof(msg), "GRN_%04d: %d+%d+%d ms", file_number, t_copy, t_blend, t_save);
        set_result(msg);
    }

    /* free the big buffer before (possibly) waiting on Canon's files */
    free(frame); frame = 0;

    if (!grain_keep_orig)
        delete_originals(file_number);

done:
    if (pf) FIO_CloseFile(pf);
    if (plate_strip) free(plate_strip);
    if (plate_row) free(plate_row);
    if (packed_row) free(packed_row);
    if (row) free(row);
    if (frame) free(frame);
    job_busy = 0;
}

/* ------------------------------------------------------------------ */
/* CR2 source (DIGIC 6/7)                                              */
/* ------------------------------------------------------------------ */

struct cr2_io
{
    FILE * cr2, * plate, * out;
    uint8_t * bounce;           /* DMA; FIO reads land here, then memcpy to the decoder chunk */
    uint32_t cr2_pos;           /* skip the seek for sequential reads */
};

static int cr2_read(void * ctx, uint32_t off, void * buf, int n)
{
    struct cr2_io * io = ctx;
    int done = 0;
    while (done < n)
    {
        int m = MIN(n - done, CR2_CHUNK);
        if (io->cr2_pos != off + done)
        {
            /* returns the new position (550D); only treat negative values as errors,
             * the return value of the DIGIC 6 stub is unverified */
            if (FIO_SeekSkipFile(io->cr2, off + done, SEEK_SET) < 0)
                return done;
            io->cr2_pos = off + done;
        }
        int r = FIO_ReadFile(io->cr2, io->bounce, m);
        if (r != m)
        {
            io->cr2_pos = 0xFFFFFFFF;
            return done;
        }
        memcpy((uint8_t *) buf + done, io->bounce, m);
        io->cr2_pos += m;
        done += m;
    }
    return done;
}

static int cr2_read_plate(void * ctx, void * buf, int n)
{
    return FIO_ReadFile(((struct cr2_io *) ctx)->plate, buf, n);
}

static int cr2_write(void * ctx, const void * buf, int n)
{
    return FIO_WriteFile(((struct cr2_io *) ctx)->out, buf, n);
}

static void cr2_yield(void * ctx)
{
    /* low priority task, but DryOS does not time-slice equal priorities;
     * sleep after ~100 ms of work (msleep(10) is 11 ms on 750D) */
    static int last = 0;
    int now = get_ms_clock();
    if (now - last >= 100)
    {
        msleep(10);
        last = get_ms_clock();
    }
}

static struct grn_cr2 cr2_hdr;              /* ~6 KB, keep it off the task stack */
static struct grn_cr2_job * volatile cr2_job = 0;
static volatile int cr2_watch_running = 0;

/* find IMG_nnnn.CR2 (or _MG_ for Adobe RGB) in the current DCIM folder */
static int cr2_path(int file_number, char * fn, int size, uint32_t * fsize)
{
    const char * prefixes[] = { get_file_prefix(), "IMG_", "_MG_" };
    for (int i = 0; i < 3; i++)
    {
        snprintf(fn, size, "%s/%s%04d.CR2", get_dcim_dir(), prefixes[i], file_number);
        if (FIO_GetFileSize(fn, fsize) == 0) return 1;
    }
    return 0;
}

static const char * cr2_strerror(int err)
{
    switch (err)
    {
        case GRN_CR2_ERR_READ:     return "CR2 read error";
        case GRN_CR2_ERR_FORMAT:   return "not a CR2";
        case GRN_CR2_ERR_JPEG:     return "unsupported CR2 (sRAW/mRAW?)";
        case GRN_CR2_ERR_DATA:     return "CR2 data corrupt";
        case GRN_CR2_ERR_GEOMETRY: return "unexpected CR2 geometry";
        case GRN_CR2_ERR_PLATE:    return "plate read error";
        case GRN_CR2_ERR_WRITE:    return "DNG write failed (card full?)";
        default:                   return "error";
    }
}

/* wait until Canon has finished writing the CR2 (size stable for 0.4 s); max_wait in ms */
static int cr2_wait_file(int file_number, char * fn, int size, uint32_t * fsize, int max_wait)
{
    uint32_t last = 0;
    int stable = 0;
    for (int t = 0; t < max_wait; t += 200)
    {
        uint32_t s = 0;
        if (cr2_path(file_number, fn, size, &s) && s > 0)
        {
            stable = (s == last) ? stable + 1 : 0;
            last = s;
            if (stable >= 2)
            {
                *fsize = s;
                return 1;
            }
        }
        msleep(200);
    }
    return 0;
}

static void cr2_process(int file_number, int max_wait)
{
    struct cr2_io io = { 0 };
    struct grn_cr2_job * j = 0;
    uint8_t * scratch = 0;
    uint8_t * hdr = 0;
    char cr2_fn[64], dng_fn[64], msg[48];
    int ok = 0;

    log_len = 0;
    LOG("CR2 source, file %d, dcim '%s', prefix '%s', digic %d",
        file_number, get_dcim_dir(), get_file_prefix(), get_digic_version());

    uint32_t fsize = 0;
    if (!cr2_wait_file(file_number, cr2_fn, sizeof(cr2_fn), &fsize, max_wait))
    {
        snprintf(msg, sizeof(msg), "no CR2 for %04d (RAW on?)", file_number);
        set_result(msg);
        goto done;
    }
    LOG("%s: %d bytes", cr2_fn, fsize);

    int t0 = get_ms_clock();
    io.bounce = dma_malloc(CR2_CHUNK);
    scratch   = malloc(2048);
    j         = malloc(sizeof(*j));
    io.cr2    = FIO_OpenFile(cr2_fn, O_RDONLY | O_SYNC);
    if (!io.bounce || !scratch || !j || !io.cr2)
    {
        set_result(io.cr2 ? "not enough memory" : "cannot open CR2");
        goto done;
    }
    memset(j, 0, sizeof(*j));
    io.cr2_pos = 0;

    struct grn_cr2 * c = &cr2_hdr;
    int r = grn_cr2_parse(c, &io, cr2_read, scratch, 2048);
    if (r)
    {
        set_result(cr2_strerror(r));
        goto done;
    }
    LOG("'%s' %dx%d, %d bit, ljpeg %dx%dx%d, slices %d/%d/%d, data %d..%d",
        c->model, c->width, c->height, c->bits, c->jw, c->jh, c->clrs,
        c->slice_n, c->slice_w, c->slice_last, c->data_off, c->data_end);
    LOG("active %d,%d-%d,%d, black %d, white %d, wb %d %d %d %d, iso %d",
        c->x1, c->y1, c->x2, c->y2, c->black, c->white, c->wb[0], c->wb[1], c->wb[2], c->wb[3], c->iso);

    if (c->data_end > fsize)
    {
        set_result("CR2 incomplete");
        goto done;
    }

    cam_active_w = c->x2 - c->x1;
    cam_active_h = c->y2 - c->y1;

    j->c = c;
    j->strength = grain_strength;
    j->mode = grain_mode;
    if (j->strength > 0)
    {
        if (!plate_ok)
        {
            set_result("plate not loaded");
            goto done;
        }
        if (plate_w != cam_active_w || plate_h != cam_active_h)
        {
            snprintf(msg, sizeof(msg), "plate %dx%d, need %dx%d", plate_w, plate_h, cam_active_w, cam_active_h);
            set_result(msg);
            NotifyBox(3000, "Grain: plate is %dx%d,\nneeds %dx%d", plate_w, plate_h, cam_active_w, cam_active_h);
            goto done;
        }
        j->plate_w = plate_w; j->plate_h = plate_h;
        j->plate_black = plate_black; j->plate_white = plate_white;
        j->plate_row   = malloc(plate_w * 2);
        j->plate_strip = dma_malloc(plate_w * 2 * CR2_STRIP_ROWS);
        io.plate       = FIO_OpenFile(PLATE_PATH, O_RDONLY | O_SYNC);
        if (!j->plate_row || !j->plate_strip || !io.plate)
        {
            set_result("plate open failed");
            goto done;
        }
        int64_t sp = FIO_SeekSkipFile(io.plate, GRN_HEADER_SIZE, SEEK_SET);
        LOG("plate seek -> %d", (int) sp);
        if (sp < 0)
        {
            set_result("plate seek failed");
            goto done;
        }
    }

    const int pitch = c->width * 14 / 8;
    for (int k = 0; k < c->nslices; k++)
        if (!(j->in[k] = malloc(CR2_CHUNK))) goto nomem;
    j->in_size    = CR2_CHUNK;
    j->row        = malloc(c->width * 2);
    j->packed_row = malloc(pitch);
    j->out        = dma_malloc(pitch * CR2_STRIP_ROWS);
    j->strip_rows = CR2_STRIP_ROWS;
    hdr           = dma_malloc(GRN_DNG_HEADER_MAX);
    if (!j->row || !j->packed_row || !j->out || !hdr) goto nomem;

    j->ctx = &io;
    j->read_cr2 = cr2_read;
    j->read_plate = cr2_read_plate;
    j->write_out = cr2_write;
    j->yield = cr2_yield;

    snprintf(dng_fn, sizeof(dng_fn), "%s/GRN_%04d.DNG", get_dcim_dir(), file_number);
    io.out = FIO_CreateFile(dng_fn);
    if (!io.out)
    {
        set_result("cannot create DNG");
        goto done;
    }

    cr2_job = j;
    r = grn_cr2_to_dng(j, hdr, GRN_DNG_HEADER_MAX);
    cr2_job = 0;
    FIO_CloseFile(io.out); io.out = 0;

    if (r)
    {
        FIO_RemoveFile(dng_fn);
        LOG("conversion failed: %d", r);
        set_result(cr2_strerror(r));
        NotifyBox(2000, "Grain: %s", cr2_strerror(r));
        goto done;
    }

    snprintf(msg, sizeof(msg), "GRN_%04d: %d ms", file_number, get_ms_clock() - t0);
    set_result(msg);
    ok = 1;
    goto done;

nomem:
    set_result("not enough memory");
    NotifyBox(2000, "Grain: not enough memory");

done:
    cr2_job = 0;
    if (io.out)   FIO_CloseFile(io.out);
    if (io.plate) FIO_CloseFile(io.plate);
    if (io.cr2)   FIO_CloseFile(io.cr2);
    if (j)
    {
        for (int k = 0; k < GRN_CR2_MAX_SLICES; k++) if (j->in[k]) free(j->in[k]);
        if (j->row)         free(j->row);
        if (j->packed_row)  free(j->packed_row);
        if (j->out)         free(j->out);
        if (j->plate_row)   free(j->plate_row);
        if (j->plate_strip) free(j->plate_strip);
        free(j);
    }
    if (hdr)       free(hdr);
    if (scratch)   free(scratch);
    if (io.bounce) free(io.bounce);

    log_flush();

    if (ok && !grain_keep_orig)
        delete_originals(file_number);
}

/* highest xxx_nnnn.CR2 number in the current DCIM folder, 0 if none or no dir API */
static int cr2_scan_newest(void)
{
    struct fio_file * file = alloc_fio_file();
    if (!file) return 0;

    int best = 0;
    struct fio_dirent * dirent = FIO_FindFirstEx(get_dcim_dir(), file);
    if (!IS_ERROR(dirent))
    {
        do
        {
            struct file_info fi = convert_fio_file_info(file);
            const char * s = fi.name;
            if (strlen(s) != 12 || s[8] != '.' || (s[9] | 0x20) != 'c' || (s[10] | 0x20) != 'r' || s[11] != '2')
                continue;
            int n = 0;
            for (int i = 4; i < 8 && n >= 0; i++)
                n = (s[i] >= '0' && s[i] <= '9') ? n * 10 + s[i] - '0' : -1;
            if (n > best) best = n;
        }
        while (FIO_FindNextEx(dirent, file) == 0);
        FIO_FindClose(dirent);
    }
    free(file);
    return best;
}

/*
 * Watches the file number and converts every new picture, oldest first.
 * Also probes for IMG_(last+1).CR2 directly (start: highest CR2 in the folder),
 * in case the file number property is not hooked up on a new port.
 */
static void cr2_watch_task(void * unused)
{
    int last_done = -1;
    char fn[64];

    while (cr2_watch_running)
    {
        msleep(300);

        if (!grain_enabled || !use_cr2() || !source_supported())
        {
            last_done = -1;
            continue;
        }

        int newest = get_shooting_card()->file_number;
        if (last_done < 0)
        {
            /* only pictures taken from now on */
            int scanned = cr2_scan_newest();
            last_done = MAX(newest, scanned);
            log_len = 0;
            LOG("CR2 watch: file number %d, newest CR2 in %s: %d", newest, get_dcim_dir(), scanned);
            log_flush();
            continue;
        }

        uint32_t s;
        if (newest == last_done && cr2_path(last_done + 1, fn, sizeof(fn), &s))
            newest = last_done + 1;
        if (newest == last_done)
            continue;
        if (newest < last_done)
            last_done = newest - 1;       /* numbering wrapped or was reset */
        if (newest - last_done > 10)
            last_done = newest - 10;      /* long burst: keep the latest 10 */

        while (last_done < newest && cr2_watch_running && grain_enabled)
        {
            int n = ++last_done;
            job_busy = 1;
            /* the newest file may still be exposing/writing (long exposures) */
            cr2_process(n, n == newest ? 120000 : 2000);
            job_busy = 0;
        }
    }
    cr2_watch_running = -1;
}

PROP_HANDLER(PROP_GUI_STATE)
{
    if (!grain_enabled) return;
    if (use_cr2() || !source_supported()) return;

    uint32_t * data = buf;
    if (data[0] != GUISTATE_QR) return;

    if (job_busy)
    {
        /* one frame buffer at a time (550D memory); tell the user this shot was skipped */
        if (get_shooting_card()->file_number != last_file_number)
            NotifyBox(1500, "Grain: busy, shot skipped");
        return;
    }

    job_busy = 1;
    /* never do the work in the property task; low priority, like deflick */
    task_create("grain_task", 0x1a, 0x2000, grain_task, 0);
}

/* ------------------------------------------------------------------ */
/* menu                                                                */
/* ------------------------------------------------------------------ */

static MENU_UPDATE_FUNC(grain_main_update)
{
    if (!grain_enabled) return;

    if (!source_supported())
        MENU_SET_WARNING(MENU_WARN_NOT_WORKING, use_cr2()
            ? "DIGIC %d saves CR3, the CR2 source can't read it."
            : "DIGIC %d: no ML raw buffer access. Set Source to CR2 file.", get_digic_version());
    else if (grain_strength > 0 && !plate_ok)
        MENU_SET_WARNING(MENU_WARN_NOT_WORKING, "Plate: %s", plate_err);
    else if (use_cr2())
        /* pic_quality constants are not verified on DIGIC 6+; a missing CR2 shows up in Last result */
        MENU_SET_WARNING(MENU_WARN_INFO, "CR2 source: picture quality RAW or RAW+JPEG, then shoot normally.");
    else if (!can_use_raw_overlays_photo())
        MENU_SET_WARNING(MENU_WARN_NOT_WORKING, "Set picture quality to RAW or RAW+JPEG.");
    else if (image_review_time == 0)
        MENU_SET_WARNING(MENU_WARN_NOT_WORKING, "Enable image review from Canon menu.");
    else if (is_continuous_drive())
        MENU_SET_WARNING(MENU_WARN_ADVICE, "Continuous drive: shots during processing are skipped.");

    MENU_SET_RINFO("%d%% %s", grain_strength,
        grain_mode == GRN_BLEND_SCREEN ? "Screen" : grain_mode == GRN_BLEND_LIGHTEN ? "Lighten" : "Overlay");
}

static MENU_UPDATE_FUNC(plate_status_update)
{
    if (plate_ok)
        MENU_SET_VALUE("%dx%d", plate_w, plate_h);
    else
        MENU_SET_VALUE("%s", plate_err);

    if (cam_active_w)
    {
        MENU_SET_RINFO("cam %dx%d", cam_active_w, cam_active_h);
        if (plate_ok && (plate_w != cam_active_w || plate_h != cam_active_h))
            MENU_SET_WARNING(MENU_WARN_NOT_WORKING, "Plate must be %dx%d (camera active area).", cam_active_w, cam_active_h);
    }
    else
    {
        MENU_SET_WARNING(MENU_WARN_INFO, "Take one picture to detect the required plate size.");
    }
}

static MENU_SELECT_FUNC(plate_reload)
{
    plate_load_header();
}

static MENU_UPDATE_FUNC(last_result_update)
{
    MENU_SET_VALUE("%s", last_result[0] ? last_result : "-");
    struct grn_cr2_job * j = cr2_job;
    if (j) MENU_SET_RINFO("busy %d%%", j->progress);
    else if (job_busy) MENU_SET_RINFO("busy");
}

static MENU_UPDATE_FUNC(source_update)
{
    if (grain_source == SOURCE_AUTO)
        MENU_SET_RINFO("%s", use_cr2() ? "CR2 file" : "raw buffer");
}

static MENU_UPDATE_FUNC(keep_update)
{
    if (!grain_keep_orig)
        MENU_SET_WARNING(MENU_WARN_ADVICE, "Experimental: CR2/JPG are deleted after a good DNG.");
}

static struct menu_entry grain_menu[] =
{
    {
        .name = "Grain Overlay",
        .priv = &grain_enabled,
        .max = 1,
        .update = grain_main_update,
        .works_best_in = DEP_PHOTO_MODE,
        .help  = "Blend a film grain plate (ML/GRAIN.GRN) into each photo.",
        .help2 = "Output: GRN_nnnn.DNG next to the CR2. Needs RAW or RAW+JPEG.",
        .submenu_width = 710,
        .children = (struct menu_entry[]) {
            {
                .name = "Strength",
                .priv = &grain_strength,
                .min = 0,
                .max = 100,
                .unit = UNIT_PERCENT,
                .help  = "Plate intensity. 0% = untouched copy of the raw data",
                .help2 = "(useful to verify the DNG round trip).",
            },
            {
                .name = "Blend mode",
                .priv = &grain_mode,
                .max = 2,
                .choices = CHOICES("Screen", "Lighten", "Overlay"),
                .help  = "Screen/Lighten: plate is premultiplied by strength.",
                .help2 = "Overlay: plate should be centered at 50% gray.",
            },
            {
                .name = "Source",
                .priv = &grain_source,
                .max = 2,
                .choices = CHOICES("Auto", "Raw buffer", "CR2 file"),
                .update = source_update,
                .help  = "Raw buffer: copy during image review (DIGIC 4/5, needs review).",
                .help2 = "CR2 file: decode Canon's CR2 after saving (DIGIC 6/7, slower).",
            },
            {
                .name = "Save original too",
                .priv = &grain_keep_orig,
                .max = 1,
                .update = keep_update,
                .help  = "OFF: delete Canon's CR2/JPG after the DNG was saved OK.",
            },
            {
                .name = "Plate",
                .select = plate_reload,
                .update = plate_status_update,
                .icon_type = IT_ACTION,
                .help  = "Status of ML/GRAIN.GRN. Press SET to reload from card.",
            },
            {
                .name = "Last result",
                .update = last_result_update,
                .icon_type = IT_ACTION,
                .help  = "Result of the last processed picture (times in ms).",
            },
            {
                .name = "Debug log",
                .priv = &grain_debug,
                .max = 1,
                .help  = "Print details to the ML console / log.",
                .help2 = "CR2 source: also writes ML/LOGS/GRAIN.LOG after each picture.",
            },
            MENU_EOL,
        },
    },
};

static unsigned int grain_init()
{
    plate_load_header();
    menu_add("Shoot", grain_menu, COUNT(grain_menu));
    /* CR2 source; idles when disabled. Background priority: decoding takes
     * seconds of CPU and must never starve Canon's LiveView tasks */
    cr2_watch_running = 1;
    task_create("grain_cr2", 0x1e, 0x4000, cr2_watch_task, 0);
    return 0;
}

static unsigned int grain_deinit()
{
    grain_enabled = 0;
    if (cr2_watch_running == 1)
    {
        cr2_watch_running = 0;
        while (cr2_watch_running != -1) msleep(100);
    }
    while (job_busy) msleep(100);
    menu_remove("Shoot", grain_menu, COUNT(grain_menu));
    return 0;
}

MODULE_INFO_START()
    MODULE_INIT(grain_init)
    MODULE_DEINIT(grain_deinit)
MODULE_INFO_END()

MODULE_CONFIGS_START()
    MODULE_CONFIG(grain_enabled)
    MODULE_CONFIG(grain_strength)
    MODULE_CONFIG(grain_mode)
    MODULE_CONFIG(grain_keep_orig)
    MODULE_CONFIG(grain_source)
    MODULE_CONFIG(grain_debug)
MODULE_CONFIGS_END()

MODULE_PROPHANDLERS_START()
    MODULE_PROPHANDLER(PROP_GUI_STATE)
MODULE_PROPHANDLERS_END()
