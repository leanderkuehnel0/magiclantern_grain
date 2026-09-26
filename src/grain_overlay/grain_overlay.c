/**
 * grain_overlay - blend a scanned film-grain plate into still photos (DNG output)
 *
 * Target: Canon 550D (DIGIC 4). Should work on other DIGIC 4/5 bodies with
 * CONFIG_RAW_PHOTO, but only the 550D was designed for.
 *
 * USAGE
 *   1. Copy the plate to the card as  ML/GRAIN.GRN
 *   2. Canon menu: picture quality RAW or RAW+JPEG, image review ON (2s or more)
 *   3. ML menu -> Shoot -> Grain Overlay: ON
 *   4. Shoot normally. After each picture, DCIM/xxxCANON/GRN_nnnn.DNG is written
 *      (nnnn = Canon file number, so it pairs with IMG_nnnn.CR2).
 *
 * HOW IT WORKS
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
 *   not 5184x3456 - ML's active area includes a few more pixels than JPEG size).
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

#define PLATE_PATH      "ML/GRAIN.GRN"
#define STRIP_ROWS      32

static CONFIG_INT("grain.enabled",  grain_enabled,  0);
static CONFIG_INT("grain.strength", grain_strength, 50);
static CONFIG_INT("grain.mode",     grain_mode,     GRN_BLEND_SCREEN);
static CONFIG_INT("grain.keep",     grain_keep_orig, 1);
static CONFIG_INT("grain.debug",    grain_debug,    0);

#define LOG(fmt, ...) do { if (grain_debug) printf("[grain] " fmt "\n", ## __VA_ARGS__); } while (0)

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

PROP_HANDLER(PROP_GUI_STATE)
{
    if (!grain_enabled) return;

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

    if (!can_use_raw_overlays_photo())
        MENU_SET_WARNING(MENU_WARN_NOT_WORKING, "Set picture quality to RAW or RAW+JPEG.");
    else if (image_review_time == 0)
        MENU_SET_WARNING(MENU_WARN_NOT_WORKING, "Enable image review from Canon menu.");
    else if (grain_strength > 0 && !plate_ok)
        MENU_SET_WARNING(MENU_WARN_NOT_WORKING, "Plate: %s", plate_err);
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
    if (job_busy) MENU_SET_RINFO("busy");
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
        .help2 = "Output: GRN_nnnn.DNG next to the CR2. Needs RAW + image review.",
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
                .help  = "Result of the last processed picture (copy+blend+save times).",
            },
            {
                .name = "Debug log",
                .priv = &grain_debug,
                .max = 1,
                .help  = "Print details to the ML console / log.",
            },
            MENU_EOL,
        },
    },
};

static unsigned int grain_init()
{
    plate_load_header();
    menu_add("Shoot", grain_menu, COUNT(grain_menu));
    return 0;
}

static unsigned int grain_deinit()
{
    grain_enabled = 0;
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
    MODULE_CONFIG(grain_debug)
MODULE_CONFIGS_END()

MODULE_PROPHANDLERS_START()
    MODULE_PROPHANDLER(PROP_GUI_STATE)
MODULE_PROPHANDLERS_END()
