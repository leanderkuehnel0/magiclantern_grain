/* host unit test for grain_core.h:  gcc -O2 -Wall -o test_core test_core.c && ./test_core */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "../grain_core.h"

static int fails = 0;
#define CHECK(c, ...) do { if (!(c)) { fails++; printf("FAIL: " __VA_ARGS__); printf("\n"); } } while (0)

/* reference pack straight from the bit diagram in src/raw.h (16-bit BE words, byte-swapped) */
static void ref_pack8(const uint16_t *v, uint8_t *o)
{
    uint64_t hi = 0; /* build 112-bit big-endian stream */
    uint8_t be[14] = {0};
    int bit = 0;
    for (int i = 0; i < 8; i++)
        for (int k = 13; k >= 0; k--, bit++)
            if (v[i] >> k & 1) be[bit/8] |= 0x80 >> (bit%8);
    (void)hi;
    for (int w = 0; w < 7; w++) { o[2*w] = be[2*w+1]; o[2*w+1] = be[2*w]; }
}

int main(void)
{
    srand(1234);
    const int W = 5344, H = 8;               /* 550D photo raw width */
    int pitch = W * 14 / 8;
    uint8_t *packed = malloc(pitch * H), *ref = malloc(pitch * H), *packed2 = malloc(pitch * H);
    uint16_t *row = malloc(W * 2);

    /* 1. our packer matches the raw.h bit layout; unpack(pack(x)) == x */
    for (int y = 0; y < H; y++)
    {
        uint16_t vals[5344];
        for (int x = 0; x < W; x++) vals[x] = rand() & 16383;
        grn_pack_row(vals, packed + y*pitch, W);
        for (int x = 0; x < W; x += 8) ref_pack8(vals + x, ref + y*pitch + x/8*14);
        grn_unpack_row(packed + y*pitch, row, W);
        CHECK(memcmp(row, vals, W*2) == 0, "unpack(pack) row %d", y);
    }
    CHECK(memcmp(packed, ref, pitch*H) == 0, "packer differs from raw.h layout");

    /* 2. pack(unpack(bytes)) == bytes for arbitrary packed data */
    for (int i = 0; i < pitch*H; i++) packed[i] = rand();
    for (int y = 0; y < H; y++)
    {
        grn_unpack_row(packed + y*pitch, row, W);
        grn_pack_row(row, packed2 + y*pitch, W);
    }
    CHECK(memcmp(packed, packed2, pitch*H) == 0, "pack(unpack) not identity");

    /* 3. strength 0 is bit-exact for all modes, all pixel values, all plate values */
    struct grn_blend_params bp;
    for (int m = 0; m < 3; m++)
    {
        CHECK(grn_blend_setup(&bp, m, 0, 2047, 15000, 0, 16383) == 0, "setup");
        int bad = 0;
        for (int p = 0; p <= 16383; p++)
            for (int v = 0; v <= 16383; v += 127)
                if (grn_blend_pixel(&bp, p, v) != p) bad++;
        CHECK(bad == 0, "mode %d strength 0 not exact (%d)", m, bad);
    }

    /* 4. plate black (0) = no change for screen/lighten at any strength */
    for (int m = 0; m < 2; m++)
    {
        grn_blend_setup(&bp, m, 100, 2047, 15000, 0, 16383);
        int bad = 0;
        for (int p = 2047; p <= 16383; p++) if (grn_blend_pixel(&bp, p, 0) != p) bad++;
        CHECK(bad == 0, "mode %d black plate changed image (%d)", m, bad);
    }

    /* 5. overlay with 50%% gray plate = no change (within 1 LSB) */
    grn_blend_setup(&bp, GRN_BLEND_OVERLAY, 100, 2047, 15000, 0, 16383);
    { int worst = 0; for (int p = 2047; p <= 15000; p++) { int d = abs(grn_blend_pixel(&bp, p, 8192) - p); if (d > worst) worst = d; }
      CHECK(worst <= 2, "overlay neutral gray error %d", worst); printf("overlay 50%% gray max error: %d LSB\n", worst); }

    /* 6. screen lifts shadows more than highlights; white stays white; monotonic in strength */
    grn_blend_setup(&bp, GRN_BLEND_SCREEN, 50, 2047, 15000, 0, 16383);
    int lift_shadow = grn_blend_pixel(&bp, 2047 + 200, 4000) - (2047 + 200);
    int lift_high   = grn_blend_pixel(&bp, 14000, 4000) - 14000;
    printf("screen 50%%, plate 24%%: shadow lift %d, highlight lift %d\n", lift_shadow, lift_high);
    CHECK(lift_shadow > lift_high && lift_high >= 0, "screen shadow/highlight");
    CHECK(grn_blend_pixel(&bp, 15000, 16383) == 15000, "screen white not preserved: %d", grn_blend_pixel(&bp, 15000, 16383));
    { int prev = -1; for (int s = 0; s <= 100; s += 10) { grn_blend_setup(&bp, 0, s, 2047, 15000, 0, 16383);
        int o = grn_blend_pixel(&bp, 3000, 6000); CHECK(o >= prev, "not monotonic"); prev = o; } }

    /* 7. sub-black noise survives (not clamped to black) */
    grn_blend_setup(&bp, GRN_BLEND_SCREEN, 50, 2047, 15000, 0, 16383);
    CHECK(grn_blend_pixel(&bp, 1990, 0) == 1990, "sub-black pixel altered");

    /* 8. output always in 14-bit range, extreme inputs */
    for (int m = 0; m < 3; m++) for (int s = 0; s <= 100; s += 25)
    {
        grn_blend_setup(&bp, m, s, 0, 16383, 0, 16383);
        for (int p = 0; p <= 16383; p += 97) for (int v = 0; v <= 16383; v += 97)
        { int o = grn_blend_pixel(&bp, p, v); if (o < 0 || o > 16383) { CHECK(0, "range"); goto out; } }
    }
out:
    /* 9. header parser */
    uint8_t h[24] = { 'G','R','N','1', 1,0,0,0, 0x52,0x14,0,0, 0x88,0x0D,0,0, 0,0,0,0, 0xFF,0x3F,0,0 };
    int w, hh, bl, wl;
    CHECK(grn_parse_header(h, 24 + 5202u*3464u*2u, &w, &hh, &bl, &wl) == 0 && w == 5202 && hh == 3464, "header ok");
    CHECK(grn_parse_header(h, 24 + 5202u*3464u*2u - 2, &w, &hh, &bl, &wl) == -5, "size mismatch detected");
    h[3] = '2'; CHECK(grn_parse_header(h, 24 + 5202u*3464u*2u, &w, &hh, &bl, &wl) == -1, "magic");

    /* 10. setup rejects nonsense */
    CHECK(grn_blend_setup(&bp, 3, 50, 2047, 15000, 0, 16383) != 0, "bad mode accepted");
    CHECK(grn_blend_setup(&bp, 0, 50, 2047, 2100, 0, 16383) != 0, "tiny range accepted");

    printf(fails ? "\n%d FAILED\n" : "\nALL TESTS PASSED\n", fails);
    return fails != 0;
}
