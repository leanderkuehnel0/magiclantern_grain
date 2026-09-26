/*
 * grain_core.h - pure computation part of the grain_overlay module.
 *
 * No Magic Lantern / DryOS dependencies here, so this file can also be
 * compiled and unit-tested on a PC (see tests/test_core.c).
 *
 * All math is integer-only (DIGIC 4 / ARM946 has neither FPU nor a
 * hardware divider in the hot loop).
 */
#ifndef _grain_core_h_
#define _grain_core_h_

#include <stdint.h>

/* same bit layout as struct raw_pixblock in src/raw.h (8 pixels / 14 bytes) */
struct grn_pixblock
{
    unsigned int b_hi: 2;
    unsigned int a: 14;
    unsigned int c_hi: 4;
    unsigned int b_lo: 12;
    unsigned int d_hi: 6;
    unsigned int c_lo: 10;
    unsigned int e_hi: 8;
    unsigned int d_lo: 8;
    unsigned int f_hi: 10;
    unsigned int e_lo: 6;
    unsigned int g_hi: 12;
    unsigned int f_lo: 4;
    unsigned int h: 14;
    unsigned int g_lo: 2;
} __attribute__((packed,aligned(2)));

#define GRN_BLEND_SCREEN   0
#define GRN_BLEND_LIGHTEN  1
#define GRN_BLEND_OVERLAY  2

#define GRN_Q14_ONE 16384

/* plate file header (little endian, 24 bytes) */
#define GRN_HEADER_SIZE 24
#define GRN_MAGIC_0 'G'
#define GRN_MAGIC_1 'R'
#define GRN_MAGIC_2 'N'
#define GRN_MAGIC_3 '1'

/* unpack one row of 14-bit packed pixels; width must be a multiple of 8 */
static inline void grn_unpack_row(const void * packed, uint16_t * out, int width)
{
    const struct grn_pixblock * p = (const struct grn_pixblock *) packed;
    for (int x = 0; x < width; x += 8, p++)
    {
        out[x+0] = p->a;
        out[x+1] = p->b_lo | (p->b_hi << 12);
        out[x+2] = p->c_lo | (p->c_hi << 10);
        out[x+3] = p->d_lo | (p->d_hi << 8);
        out[x+4] = p->e_lo | (p->e_hi << 6);
        out[x+5] = p->f_lo | (p->f_hi << 4);
        out[x+6] = p->g_lo | (p->g_hi << 2);
        out[x+7] = p->h;
    }
}

/* pack one row back; values must be in 0..16383 */
static inline void grn_pack_row(const uint16_t * in, void * packed, int width)
{
    struct grn_pixblock * p = (struct grn_pixblock *) packed;
    for (int x = 0; x < width; x += 8, p++)
    {
        unsigned v;
        p->a = in[x+0];
        v = in[x+1]; p->b_lo = v; p->b_hi = v >> 12;
        v = in[x+2]; p->c_lo = v; p->c_hi = v >> 10;
        v = in[x+3]; p->d_lo = v; p->d_hi = v >> 8;
        v = in[x+4]; p->e_lo = v; p->e_hi = v >> 6;
        v = in[x+5]; p->f_lo = v; p->f_hi = v >> 4;
        v = in[x+6]; p->g_lo = v; p->g_hi = v >> 2;
        p->h = in[x+7];
    }
}

/* precomputed per-shot parameters */
struct grn_blend_params
{
    int mode;           /* GRN_BLEND_* */
    int strength_q14;   /* 0 .. 16384 */
    int black;          /* photo black level */
    int range;          /* photo white - black (> 0) */
    int plate_black;    /* plate black level */
    int plate_inv;      /* (16384 << 16) / (plate_white - plate_black) */
};

/* returns 0 on success; validates everything that could overflow */
static inline int grn_blend_setup(struct grn_blend_params * bp, int mode, int strength_percent,
                                  int black, int white, int plate_black, int plate_white)
{
    if (mode < GRN_BLEND_SCREEN || mode > GRN_BLEND_OVERLAY) return -1;
    if (strength_percent < 0 || strength_percent > 100) return -2;
    if (black < 0 || white > 16383 || white - black < 256) return -3;
    if (plate_black < 0 || plate_white > 16383 || plate_white - plate_black < 256) return -4;

    bp->mode = mode;
    bp->strength_q14 = strength_percent * GRN_Q14_ONE / 100;
    bp->black = black;
    bp->range = white - black;
    bp->plate_black = plate_black;
    bp->plate_inv = (GRN_Q14_ONE << 16) / (plate_white - plate_black);
    return 0;
}

/* plate sample -> Q14 (0..16384) */
static inline int grn_plate_q14(const struct grn_blend_params * bp, int v)
{
    int b = ((v - bp->plate_black) * bp->plate_inv) >> 16;
    if (b < 0) b = 0;
    if (b > GRN_Q14_ONE) b = GRN_Q14_ONE;
    return b;
}

/*
 * Blend one pixel. Works in raw units relative to black (d = p - black),
 * which keeps values below black (shadow noise) intact and makes
 * "no effect" cases (strength 0) bit-exact.
 *
 *   Screen : out = d + s*b*(R - d)            (== 1-(1-a)(1-s*b), normalized)
 *   Lighten: out = max(d, s*b*R)
 *   Overlay: ov  = overlay(a, b); out = d + s*(ov - d)
 *            (plate should be centered at 50% gray; 50% gray = no change)
 */
static inline int grn_blend_pixel(const struct grn_blend_params * bp, int p, int plate_v)
{
    const int R = bp->range;
    const int s = bp->strength_q14;
    int d = p - bp->black;
    int b = grn_plate_q14(bp, plate_v);
    int out;

    switch (bp->mode)
    {
        case GRN_BLEND_SCREEN:
        {
            int bs = (b * s) >> 14;
            out = d + ((bs * (R - d)) >> 14);
            break;
        }
        case GRN_BLEND_LIGHTEN:
        {
            int bs = (b * s) >> 14;
            int lv = (bs * R) >> 14;
            /* lv == 0 (black plate or strength 0) must not touch sub-black noise */
            out = (lv > 0 && lv > d) ? lv : d;
            break;
        }
        default: /* GRN_BLEND_OVERLAY */
        {
            int ov;
            if (2 * d < R)
                ov = (2 * d * b) >> 14;
            else
                ov = R - ((2 * (R - d) * (GRN_Q14_ONE - b)) >> 14);
            out = d + (((ov - d) * s) >> 14);
            break;
        }
    }

    out += bp->black;
    if (out < 0) out = 0;
    if (out > 16383) out = 16383;
    return out;
}

/* blend a span of one row: row[x0 .. x0+n-1] with plate[0 .. n-1] */
static inline void grn_blend_span(const struct grn_blend_params * bp, uint16_t * row, int x0,
                                  const uint16_t * plate, int n)
{
    uint16_t * r = row + x0;
    for (int i = 0; i < n; i++)
        r[i] = grn_blend_pixel(bp, r[i], plate[i]);
}

/* parse & validate the 24-byte header; returns 0 on success */
static inline int grn_parse_header(const uint8_t * h, uint32_t file_size,
                                   int * w, int * hgt, int * black, int * white)
{
    #define GRN_U32(o) ((uint32_t)h[o] | ((uint32_t)h[o+1] << 8) | ((uint32_t)h[o+2] << 16) | ((uint32_t)h[o+3] << 24))
    if (h[0] != GRN_MAGIC_0 || h[1] != GRN_MAGIC_1 || h[2] != GRN_MAGIC_2 || h[3] != GRN_MAGIC_3) return -1;
    if (GRN_U32(4) != 1) return -2;
    uint32_t ww = GRN_U32(8), hh = GRN_U32(12), bl = GRN_U32(16), wl = GRN_U32(20);
    #undef GRN_U32
    if (ww == 0 || hh == 0 || ww > 8192 || hh > 8192) return -3;
    if (bl > 16383 || wl > 16383 || wl < bl + 256) return -4;
    if (file_size != GRN_HEADER_SIZE + ww * hh * 2) return -5;
    *w = ww; *hgt = hh; *black = bl; *white = wl;
    return 0;
}

#endif
