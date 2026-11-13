/*
 * host test for grain_cr2.h (the DIGIC 6 path), on a real CR2:
 *
 *   curl -LO 'https://raw.pixls.us/data/Canon/EOS%20750D/IMG_5474.CR2'
 *   cc -O2 -Wall -o test_cr2 test_cr2.c
 *   dcraw -E -4 -t 0 -c IMG_5474.CR2 > ref.pgm   (raw sensor values, no crop)
 *   ./test_cr2 IMG_5474.CR2 ref.pgm out_dir
 *
 * 1. strength 0: the DNG pixel data must equal dcraw's decode, bit for bit
 * 2. screen 50% with a noise plate: every active pixel must equal
 *    grn_blend_pixel(reference, plate), everything outside the active area
 *    must be untouched
 * Both runs use camera-sized buffers (64 KB input chunks, 16-row strips).
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../grain_cr2.h"

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

struct io
{
    FILE * cr2;
    FILE * out;
    const uint16_t * plate;
    size_t plate_pos, plate_bytes;
    long pos;
    int reads, seeks, yields;
};

/* one handle per slice decoder (like the camera); count seeks = non-sequential reads */
static int rd_cr2(void * ctx, uint32_t off, void * buf, int n)
{
    struct io * io = ctx;
    io->reads++;
    if (off != io->pos) io->seeks++;
    if (fseek(io->cr2, off, SEEK_SET)) return -1;
    int r = fread(buf, 1, n, io->cr2);
    io->pos = off + r;
    return r;
}

static int rd_plate(void * ctx, void * buf, int n)
{
    struct io * io = ctx;
    if (io->plate_pos + n > io->plate_bytes) return -1;
    memcpy(buf, (const uint8_t *) io->plate + io->plate_pos, n);
    io->plate_pos += n;
    return n;
}

static int wr_out(void * ctx, const void * buf, int n)
{
    return fwrite(buf, 1, n, ((struct io *) ctx)->out);
}

static void yield(void * ctx) { ((struct io *) ctx)->yields++; }

static uint16_t * load_pgm(const char * fn, int * w, int * h)
{
    FILE * f = fopen(fn, "rb");
    if (!f) return 0;
    int maxv;
    if (fscanf(f, "P5 %d %d %d", w, h, &maxv) != 3) return 0;
    fgetc(f);
    size_t n = (size_t) *w * *h;
    uint16_t * p = malloc(n * 2);
    uint8_t * b = malloc(n * 2);
    if (fread(b, 2, n, f) != n) return 0;
    for (size_t i = 0; i < n; i++) p[i] = b[2*i] << 8 | b[2*i+1];
    free(b); fclose(f);
    return p;
}

/* read our own DNG back: header size from StripOffsets, then unpack rows */
static uint16_t * load_dng_pixels(const char * fn, int w, int h)
{
    FILE * f = fopen(fn, "rb");
    if (!f) return 0;
    static uint8_t hdr[GRN_DNG_HEADER_MAX];
    fread(hdr, 1, sizeof(hdr), f);
    int n = hdr[8] << 8 | hdr[9];
    uint32_t strip = 0;
    for (int i = 0; i < n; i++)
    {
        uint8_t * e = hdr + 10 + 12 * i;
        if ((e[0] << 8 | e[1]) == 0x0111) strip = (uint32_t) e[8] << 24 | e[9] << 16 | e[10] << 8 | e[11];
    }
    int pitch = w * 14 / 8;
    uint16_t * p = malloc((size_t) w * h * 2);
    uint8_t * row = malloc(pitch);
    fseek(f, strip, SEEK_SET);
    for (int y = 0; y < h; y++)
    {
        if (fread(row, 1, pitch, f) != (size_t) pitch) { free(p); p = 0; break; }
        grn_row_to_dng(row, pitch);               /* swap back to Canon layout */
        grn_unpack_row(row, p + (size_t) y * w, w);
    }
    free(row); fclose(f);
    return p;
}

static int run(struct grn_cr2 * c, const char * cr2_fn, const char * out_fn,
               int strength, int mode, const uint16_t * plate, int pw, int ph)
{
    struct io io = { 0 };
    io.cr2 = fopen(cr2_fn, "rb");
    io.out = fopen(out_fn, "wb");
    io.plate = plate; io.plate_bytes = (size_t) pw * ph * 2;

    static uint8_t scratch[2048];
    int r = grn_cr2_parse(c, &io, rd_cr2, scratch, sizeof(scratch));
    if (r) { printf("parse error %d\n", r); return r; }

    const int strip = 16, in_size = 64 * 1024;
    struct grn_cr2_job j = { 0 };
    j.c = c; j.strength = strength; j.mode = mode;
    j.plate_w = pw; j.plate_h = ph; j.plate_black = 0; j.plate_white = 16383;
    for (int k = 0; k < c->nslices; k++) j.in[k] = malloc(in_size);
    j.in_size = in_size;
    j.row = malloc(c->width * 2);
    j.plate_row = malloc(pw * 2 + 2);
    j.out = malloc(strip * c->width * 14 / 8);
    j.plate_strip = malloc(2 * strip * pw * 2 + 2);
    j.strip_rows = strip;
    j.plate_rows = 2 * strip;
    if (strength) j.packed_row = malloc(c->width * 14 / 8);   /* camera path: pack, then copy to the DMA strip */
    j.ctx = &io; j.read_cr2 = rd_cr2; j.read_plate = rd_plate; j.write_out = wr_out; j.yield = yield;
    struct io sl[GRN_CR2_MAX_SLICES] = { { 0 } };
    for (int k = 0; k < c->nslices; k++) { sl[k].cr2 = fopen(cr2_fn, "rb"); j.slice_ctx[k] = &sl[k]; }

    static uint8_t hdr[GRN_DNG_HEADER_MAX];
    r = grn_cr2_to_dng(&j, hdr, sizeof(hdr));
    int reads = 0, seeks = 0;
    for (int k = 0; k < c->nslices; k++) { reads += sl[k].reads; seeks += sl[k].seeks; fclose(sl[k].cr2); }
    printf("%s: result %d, %d chunk reads, %d seeks, %d yields, progress %d%%\n", out_fn, r, reads, seeks, io.yields, j.progress);
    CHECK(r || j.progress == 100, "progress ended at %d%%", j.progress);
    /* one seek per slice decoder to its start (slice 0 re-reads from the scan start) */
    CHECK(r || seeks <= c->nslices + 1, "%d seeks, expected sequential reads per slice", seeks);
    fclose(io.cr2); fclose(io.out);
    return r;
}

int main(int argc, char ** argv)
{
    if (argc < 4) { printf("usage: %s IMG.CR2 ref.pgm out_dir\n", argv[0]); return 2; }
    char fn0[512], fn1[512];
    snprintf(fn0, sizeof(fn0), "%s/strength0.dng", argv[3]);
    snprintf(fn1, sizeof(fn1), "%s/screen50.dng", argv[3]);

    int rw, rh;
    uint16_t * ref = load_pgm(argv[2], &rw, &rh);
    CHECK(ref, "cannot load %s", argv[2]);
    if (!ref) return 1;

    /* 1. passthrough */
    static struct grn_cr2 c;
    int r = run(&c, argv[1], fn0, 0, 0, 0, 0, 0);
    CHECK(r == 0, "strength 0 run failed (%d)", r);
    printf("model '%s' %dx%d, %d bits, jpeg %dx%d x%d, slices %d (%d,%d,%d)\n",
           c.model, c.width, c.height, c.bits, c.jw, c.jh, c.clrs, c.nslices, c.slice_n, c.slice_w, c.slice_last);
    printf("active %d,%d-%d,%d  black %d white %d  wb %d %d %d %d  iso %d  %u/%u s  f/%u/%u  %u/%u mm  '%s' orient %d\n",
           c.x1, c.y1, c.x2, c.y2, c.black, c.white, c.wb[0], c.wb[1], c.wb[2], c.wb[3],
           c.iso, c.exp_num, c.exp_den, c.fnum_num, c.fnum_den, c.focal_num, c.focal_den, c.datetime, c.orientation);
    CHECK(c.width == rw && c.height == rh, "size %dx%d vs dcraw %dx%d", c.width, c.height, rw, rh);
    if (r || c.width != rw || c.height != rh) return 1;

    CHECK(c.black > 0 && c.white > c.black, "makernote black/white not found");
    CHECK(c.wb[0] > 0 && c.wb[3] > 0, "makernote as-shot WB not found");

    /* black level fallback (cameras whose makernote we can't read): measure the masked columns */
    {
        struct io io = { 0 };
        io.cr2 = fopen(argv[1], "rb");
        struct grn_cr2 c2 = c;
        c2.black = 0;
        struct grn_cr2_job j = { 0 };
        j.c = &c2; j.ctx = &io; j.read_cr2 = rd_cr2;
        j.in[0] = malloc(64 * 1024); j.in_size = 64 * 1024;
        j.row = malloc(c.width * 2);
        struct grn_ljstate st[GRN_CR2_MAX_SLICES];
        int ri = grn_cr2_index(&j, st);
        printf("black: makernote %d, measured on masked columns %d\n", c.black, c2.black);
        CHECK(ri == 0 && abs(c2.black - c.black) <= 16, "measured black %d vs makernote %d", c2.black, c.black);
        fclose(io.cr2);
    }

    uint16_t * got = load_dng_pixels(fn0, rw, rh);
    CHECK(got, "cannot read back %s", fn0);
    size_t n = (size_t) rw * rh, bad = 0;
    for (size_t i = 0; i < n; i++)
        if (got[i] != ref[i] && bad++ < 5)
            printf("  pixel %zu (%zu,%zu): got %d want %d\n", i, i % rw, i / rw, got[i], ref[i]);
    CHECK(bad == 0, "strength 0: %zu of %zu pixels differ from dcraw", bad, n);
    printf("strength 0: %zu pixels compared, %zu differ\n", n, bad);
    free(got);

    /* 2. blend with a noise plate covering the active area */
    const int pw = c.x2 - c.x1, ph = c.y2 - c.y1;
    uint16_t * plate = malloc((size_t) pw * ph * 2);
    srand(42);
    for (size_t i = 0; i < (size_t) pw * ph; i++) plate[i] = rand() & 16383;
    struct grn_blend_params bp;
    CHECK(grn_blend_setup(&bp, GRN_BLEND_SCREEN, 50, c.black, c.white, 0, 16383) == 0, "blend setup");

    r = run(&c, argv[1], fn1, 50, GRN_BLEND_SCREEN, plate, pw, ph);
    CHECK(r == 0, "screen 50 run failed (%d)", r);
    got = load_dng_pixels(fn1, rw, rh);
    CHECK(got, "cannot read back %s", fn1);
    bad = 0;
    size_t changed = 0;
    for (int y = 0; y < rh; y++)
        for (int x = 0; x < rw; x++)
        {
            size_t i = (size_t) y * rw + x;
            int want = ref[i];
            if (x >= c.x1 && x < c.x2 && y >= c.y1 && y < c.y2)
                want = grn_blend_pixel(&bp, ref[i], plate[(size_t) (y - c.y1) * pw + (x - c.x1)]);
            if (got[i] != want && bad++ < 5) printf("  (%d,%d): got %d want %d\n", x, y, got[i], want);
            changed += got[i] != ref[i];
        }
    CHECK(bad == 0, "screen 50: %zu pixels differ from expected blend", bad);
    CHECK(changed > n / 2, "screen 50: only %zu pixels changed", changed);
    printf("screen 50: %zu pixels changed, %zu differ from expected\n", changed, bad);

    printf(fails ? "\n%d FAILURES\n" : "\nALL TESTS PASSED\n", fails);
    return fails != 0;
}
