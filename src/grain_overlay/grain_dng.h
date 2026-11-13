/*
 * grain_dng.h - minimal streaming DNG writer (header builder + row byte order).
 *
 * ML's save_dng() needs the whole frame in one contiguous buffer (and ML's
 * raw_info, which DIGIC 6 bodies do not fill for photos). This builds only the
 * header; the caller then streams the pixel rows (14-bit packed, big-endian
 * bit order).
 *
 * Layout: "MM" TIFF, IFD0 = main raw image (CFA), plus an Exif IFD.
 * Tag set follows src/chdk-dng.c (the DNGs ML already writes).
 * No ML dependencies: also compiled on the PC for tests.
 */
#ifndef _grain_dng_h_
#define _grain_dng_h_

#include <stdint.h>
#include <string.h>

struct grn_dng_info
{
    int width, height, pitch;         /* full raw frame, 14 bpp packed */
    int black, white;
    uint8_t cfa[4];                   /* 2x2 CFA at the active area origin, 0=R 1=G 2=B */
    int32_t color_matrix1[18];        /* num/den pairs (D65) */
    int active[4];                    /* y1, x1, y2, x2 */
    char model[32];
    char datetime[20];                /* "YYYY:MM:DD HH:MM:SS", empty = unknown */
    int orientation;                  /* TIFF orientation, 0 = unknown */
    int iso;                          /* 0 = unknown */
    uint32_t exp_num, exp_den;        /* exposure time; den 0 = unknown */
    uint32_t fnum_num, fnum_den;
    uint32_t focal_num, focal_den;
    uint32_t neutral[6];              /* AsShotNeutral r,g,b as num/den pairs; neutral[1] 0 = unknown */
};

#define GRN_T_BYTE      1
#define GRN_T_ASCII     2
#define GRN_T_SHORT     3
#define GRN_T_LONG      4
#define GRN_T_RATIONAL  5
#define GRN_T_UNDEF     7
#define GRN_T_SRATIONAL 10

#define GRN_DNG_MAX_ENTRIES 32
#define GRN_DNG_MAX_VAL     96
#define GRN_DNG_HEADER_MAX  2048

struct grn_ifd_entry
{
    uint16_t tag, type;
    uint32_t count;
    int nbytes;
    uint8_t val[GRN_DNG_MAX_VAL];     /* already big endian */
};

struct grn_ifd
{
    int n;
    struct grn_ifd_entry e[GRN_DNG_MAX_ENTRIES];
};

static inline void grn_be16(uint8_t * p, uint32_t v) { p[0] = v >> 8; p[1] = v; }
static inline void grn_be32(uint8_t * p, uint32_t v) { p[0] = v >> 24; p[1] = v >> 16; p[2] = v >> 8; p[3] = v; }

static inline struct grn_ifd_entry * grn_add(struct grn_ifd * ifd, uint16_t tag, uint16_t type, uint32_t count)
{
    static const int tsize[11] = { 0, 1, 1, 2, 4, 8, 0, 1, 0, 0, 8 };
    struct grn_ifd_entry * e = &ifd->e[ifd->n++];
    e->tag = tag; e->type = type; e->count = count;
    e->nbytes = tsize[type] * count;
    memset(e->val, 0, sizeof(e->val));
    return e;
}

static inline void grn_add_short(struct grn_ifd * ifd, uint16_t tag, uint32_t v)
{ grn_be16(grn_add(ifd, tag, GRN_T_SHORT, 1)->val, v); }

static inline void grn_add_long(struct grn_ifd * ifd, uint16_t tag, uint32_t v)
{ grn_be32(grn_add(ifd, tag, GRN_T_LONG, 1)->val, v); }

static inline void grn_add_rational(struct grn_ifd * ifd, uint16_t tag, uint32_t num, uint32_t den)
{
    struct grn_ifd_entry * e = grn_add(ifd, tag, GRN_T_RATIONAL, 1);
    grn_be32(e->val, num); grn_be32(e->val + 4, den);
}

static inline void grn_add_ascii(struct grn_ifd * ifd, uint16_t tag, const char * s)
{
    int n = strlen(s) + 1;
    if (n > GRN_DNG_MAX_VAL) n = GRN_DNG_MAX_VAL;
    struct grn_ifd_entry * e = grn_add(ifd, tag, GRN_T_ASCII, n);
    memcpy(e->val, s, n - 1);
}

/* count = number of elements of 'type'; b holds count * sizeof(type) big-endian bytes */
static inline void grn_add_array(struct grn_ifd * ifd, uint16_t tag, uint16_t type, uint32_t count, const uint8_t * b)
{
    struct grn_ifd_entry * e = grn_add(ifd, tag, type, count);
    memcpy(e->val, b, e->nbytes);
}

static inline struct grn_ifd_entry * grn_find(struct grn_ifd * ifd, uint16_t tag)
{
    for (int i = 0; i < ifd->n; i++) if (ifd->e[i].tag == tag) return &ifd->e[i];
    return 0;
}

static inline void grn_sort(struct grn_ifd * ifd)
{
    for (int i = 1; i < ifd->n; i++)
        for (int j = i; j > 0 && ifd->e[j-1].tag > ifd->e[j].tag; j--)
        {
            struct grn_ifd_entry t = ifd->e[j]; ifd->e[j] = ifd->e[j-1]; ifd->e[j-1] = t;
        }
}

/* serialize one IFD at 'off'; values > 4 bytes go to *data_off (advanced, word aligned) */
static inline void grn_write_ifd(uint8_t * buf, int off, struct grn_ifd * ifd, int * data_off)
{
    grn_be16(buf + off, ifd->n);
    uint8_t * p = buf + off + 2;
    for (int i = 0; i < ifd->n; i++, p += 12)
    {
        struct grn_ifd_entry * e = &ifd->e[i];
        grn_be16(p, e->tag); grn_be16(p + 2, e->type); grn_be32(p + 4, e->count);
        if (e->nbytes <= 4)
            memcpy(p + 8, e->val, 4);
        else
        {
            grn_be32(p + 8, *data_off);
            memcpy(buf + *data_off, e->val, e->nbytes);
            *data_off += (e->nbytes + 3) & ~3;
        }
    }
    grn_be32(p, 0);   /* next IFD: none */
}

/* static: ~6 KB, keep it off the (small) task stack */
static struct grn_ifd grn_ifd0, grn_exif;

/*
 * Build the header into buf (buf_size >= GRN_DNG_HEADER_MAX recommended).
 * Returns header size = offset of the pixel data (multiple of 16), or -1.
 * Pixel data that follows: height rows of pitch bytes, see grn_row_to_dng().
 */
static inline int grn_dng_build_header(uint8_t * buf, int buf_size, const struct grn_dng_info * di)
{
    struct grn_ifd * i0 = &grn_ifd0;
    struct grn_ifd * ex = &grn_exif;
    i0->n = 0; ex->n = 0;

    uint8_t tmp[GRN_DNG_MAX_VAL];

    grn_add_long (i0, 0x00FE, 0);                       /* NewSubFileType: main image */
    grn_add_long (i0, 0x0100, di->width);
    grn_add_long (i0, 0x0101, di->height);
    grn_add_short(i0, 0x0102, 14);                      /* BitsPerSample */
    grn_add_short(i0, 0x0103, 1);                       /* uncompressed */
    grn_add_short(i0, 0x0106, 32803);                   /* CFA */
    grn_add_ascii(i0, 0x010F, "Canon");
    grn_add_ascii(i0, 0x0110, di->model);
    grn_add_long (i0, 0x0111, 0);                       /* StripOffsets (patched) */
    grn_add_short(i0, 0x0112, di->orientation ? di->orientation : 1);
    grn_add_short(i0, 0x0115, 1);
    grn_add_long (i0, 0x0116, di->height);              /* one strip */
    grn_add_long (i0, 0x0117, di->pitch * di->height);
    grn_add_short(i0, 0x011C, 1);
    grn_add_ascii(i0, 0x0131, "Magic Lantern grain_overlay");
    if (di->datetime[0]) grn_add_ascii(i0, 0x0132, di->datetime);
    { uint8_t v[4]; grn_be16(v, 2); grn_be16(v + 2, 2);
      grn_add_array(i0, 0x828D, GRN_T_SHORT, 2, v); }        /* CFARepeatPatternDim 2x2 */
    grn_add_array(i0, 0x828E, GRN_T_BYTE, 4, di->cfa);
    grn_add_long (i0, 0x8769, 0);                       /* Exif IFD (patched) */
    { uint8_t v[4] = { 1, 3, 0, 0 }; grn_add_array(i0, 0xC612, GRN_T_BYTE, 4, v); }
    { uint8_t v[4] = { 1, 1, 0, 0 }; grn_add_array(i0, 0xC613, GRN_T_BYTE, 4, v); }
    grn_add_ascii(i0, 0xC614, di->model);
    grn_add_long (i0, 0xC61A, di->black);
    grn_add_long (i0, 0xC61D, di->white);
    for (int k = 0; k < 18; k++) grn_be32(tmp + 4*k, (uint32_t) di->color_matrix1[k]);
    grn_add_array(i0, 0xC621, GRN_T_SRATIONAL, 9, tmp);
    if (di->neutral[1])
    {
        for (int k = 0; k < 6; k++) grn_be32(tmp + 4*k, di->neutral[k]);
        grn_add_array(i0, 0xC628, GRN_T_RATIONAL, 3, tmp);
    }
    grn_add_short(i0, 0xC65A, 21);                      /* CalibrationIlluminant1: D65 */
    for (int k = 0; k < 4; k++) grn_be32(tmp + 4*k, di->active[k]);
    grn_add_array(i0, 0xC68D, GRN_T_LONG, 4, tmp);

    if (di->exp_den)  grn_add_rational(ex, 0x829A, di->exp_num, di->exp_den);
    if (di->fnum_den) grn_add_rational(ex, 0x829D, di->fnum_num, di->fnum_den);
    if (di->iso)      grn_add_short(ex, 0x8827, di->iso > 65535 ? 65535 : di->iso);
    { uint8_t v[4] = { '0', '2', '2', '1' }; grn_add_array(ex, 0x9000, GRN_T_UNDEF, 4, v); }
    if (di->datetime[0]) grn_add_ascii(ex, 0x9003, di->datetime);
    if (di->focal_den) grn_add_rational(ex, 0x920A, di->focal_num, di->focal_den);

    grn_sort(i0);
    grn_sort(ex);

    int off_ifd0 = 8;
    int off_exif = off_ifd0 + 2 + 12 * i0->n + 4;
    int data_off = off_exif + 2 + 12 * ex->n + 4;

    /* data area size (upper bound check) */
    int data_size = 0;
    for (int i = 0; i < i0->n; i++) if (i0->e[i].nbytes > 4) data_size += (i0->e[i].nbytes + 3) & ~3;
    for (int i = 0; i < ex->n; i++) if (ex->e[i].nbytes > 4) data_size += (ex->e[i].nbytes + 3) & ~3;
    int header_size = (data_off + data_size + 15) & ~15;
    if (header_size > buf_size) return -1;

    struct grn_ifd_entry * e_exif  = grn_find(i0, 0x8769);
    struct grn_ifd_entry * e_strip = grn_find(i0, 0x0111);
    if (!e_exif || !e_strip) return -1;
    grn_be32(e_exif->val, off_exif);
    grn_be32(e_strip->val, header_size);

    memset(buf, 0, header_size);
    buf[0] = 'M'; buf[1] = 'M'; grn_be16(buf + 2, 42); grn_be32(buf + 4, off_ifd0);
    grn_write_ifd(buf, off_ifd0, i0, &data_off);
    grn_write_ifd(buf, off_exif, ex, &data_off);
    return header_size;
}

/* Canon packs 14-bit data as little-endian 16-bit words; DNG wants a big-endian bit stream */
static inline void grn_row_to_dng(uint8_t * row, int bytes)
{
    for (int i = 0; i + 1 < bytes; i += 2)
    {
        uint8_t t = row[i]; row[i] = row[i+1]; row[i+1] = t;
    }
}

#endif
