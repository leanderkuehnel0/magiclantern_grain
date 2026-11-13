/*
 * grain_cr2.h - read the raw data back from Canon's CR2 file.
 *
 * Used on bodies where ML has no photo raw buffer access (DIGIC 6/7, e.g. 750D):
 * instead of copying the frame during image review, the module waits for the
 * CR2 on the card and decodes its lossless JPEG stream.
 *
 * Small memory: the CR2 stores the sensor as vertical slices (750D: 2 x 3048
 * columns), one whole slice after the other. Pass 1 decodes up to the start of
 * the last slice and snapshots the decoder state at every slice start. Pass 2
 * runs one decoder per slice side by side, so full sensor rows come out top to
 * bottom and can be blended and written as DNG rows one strip at a time.
 * Peak memory: one input chunk per slice + a few strip/row buffers (~0.5 MB).
 *
 * No ML / DryOS dependencies: also compiled on the PC (tests/test_cr2.c).
 * Format details follow dcraw (ljpeg_start, ljpeg_row, lossless_jpeg_load_raw)
 * and exiftool (Canon SensorInfo / ColorData tables).
 */
#ifndef _grain_cr2_h_
#define _grain_cr2_h_

#include <stdint.h>
#include <string.h>
#include "grain_core.h"
#include "grain_dng.h"

#define GRN_CR2_MAX_SLICES  4
#define GRN_HUFF_LUT_BITS   9
#define GRN_FAST_BITS       11      /* code + difference bits in one lookup (4 KB per table) */

#define GRN_CR2_ERR_READ      -10
#define GRN_CR2_ERR_FORMAT    -11   /* not a CR2 / missing tags */
#define GRN_CR2_ERR_JPEG      -12   /* unsupported lossless JPEG variant (sRAW, predictor...) */
#define GRN_CR2_ERR_DATA      -13   /* corrupt entropy-coded data */
#define GRN_CR2_ERR_GEOMETRY  -14
#define GRN_CR2_ERR_PLATE     -15
#define GRN_CR2_ERR_WRITE     -16

/* positional read into a cached buffer; returns bytes read */
typedef int (*grn_read_fn)(void * ctx, uint32_t off, void * buf, int n);

struct grn_huff
{
    int16_t  fast[1 << GRN_FAST_BITS];      /* (diff << 4) | total bits, when code + diff bits fit; 0 = use lut */
    uint16_t lut[1 << GRN_HUFF_LUT_BITS];   /* (len << 8) | value; 0 = code longer than LUT */
    int32_t  maxcode[17];                   /* largest code of each length, -1 = none */
    int32_t  valoff[17];                    /* index into vals = code + valoff[len] */
    uint8_t  vals[256];
    int      valid;
};

struct grn_cr2
{
    /* IFD0 / Exif */
    char model[32];
    char datetime[20];
    int orientation;
    int iso;
    uint32_t exp_num, exp_den, fnum_num, fnum_den, focal_num, focal_den;

    /* raw IFD */
    uint32_t data_off, data_end;            /* lossless JPEG strip */
    int slice_n, slice_w, slice_last;       /* RawImageSegmentation; slice_n = 0: single slice */
    uint8_t cfa[4];                         /* CFA at sensor (0,0), DNG codes 0=R 1=G 2=B */

    /* makernote (0 = unknown) */
    int x1, y1, x2, y2;                     /* active area, exclusive end (SensorInfo) */
    int black, white;                       /* PerChannelBlackLevel, SpecularWhiteLevel */
    int wb[4];                              /* WB_RGGBLevelsAsShot */

    /* lossless JPEG */
    int bits, jw, jh, clrs, psv;
    uint8_t comp_tbl[4];
    struct grn_huff huff[4];
    uint32_t scan_off;                      /* first entropy-coded byte */

    /* sensor frame */
    int width, height;
    int nslices;
    int slice_x[GRN_CR2_MAX_SLICES], slice_wd[GRN_CR2_MAX_SLICES];
};

/* ------------------------------------------------------------------ */
/* TIFF / makernote                                                    */
/* ------------------------------------------------------------------ */

static inline uint32_t grn_le16(const uint8_t * p) { return p[0] | (p[1] << 8); }
static inline uint32_t grn_le32(const uint8_t * p) { return p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t) p[3] << 24); }

struct grn_tiff_rd
{
    void * ctx;
    grn_read_fn read;
    uint8_t * buf;      /* scratch, >= 1024 bytes */
    int buf_size;
};

/* one IFD entry; value pointer is resolved for small values, offset otherwise */
struct grn_tag
{
    int tag, type;
    uint32_t count, value;      /* value: inline data (<= 4 bytes) or file offset */
};

static inline int grn_tiff_size(int type)
{
    static const uint8_t s[13] = { 0, 1, 1, 2, 4, 8, 1, 1, 2, 4, 8, 4, 8 };
    return (type > 0 && type < 13) ? s[type] : 0;
}

/* read up to max entries of the IFD at off; returns entry count or < 0 */
static inline int grn_tiff_ifd(struct grn_tiff_rd * t, uint32_t off, struct grn_tag * e, int max)
{
    uint8_t * b = t->buf;
    if (t->read(t->ctx, off, b, 2) != 2) return GRN_CR2_ERR_READ;
    int n = grn_le16(b);
    if (n > max) n = max;
    if (n * 12 > t->buf_size) return GRN_CR2_ERR_FORMAT;
    if (t->read(t->ctx, off + 2, b, n * 12) != n * 12) return GRN_CR2_ERR_READ;
    for (int i = 0; i < n; i++)
    {
        const uint8_t * p = b + 12 * i;
        e[i].tag = grn_le16(p);
        e[i].type = grn_le16(p + 2);
        e[i].count = grn_le32(p + 4);
        e[i].value = grn_le32(p + 8);
    }
    return n;
}

/* first value of a SHORT/LONG tag */
static inline uint32_t grn_tag_uint(const struct grn_tag * e)
{
    return e->type == 3 ? (e->value & 0xFFFF) : e->value;
}

static inline int grn_tag_read(struct grn_tiff_rd * t, const struct grn_tag * e, void * out, int max)
{
    int n = grn_tiff_size(e->type) * e->count;
    if (n > max) n = max;
    if (n <= 4) { uint8_t v[4]; v[0] = e->value; v[1] = e->value >> 8; v[2] = e->value >> 16; v[3] = e->value >> 24; memcpy(out, v, n); return n; }
    return t->read(t->ctx, e->value, out, n) == n ? n : GRN_CR2_ERR_READ;
}

static inline void grn_tag_rational(struct grn_tiff_rd * t, const struct grn_tag * e, uint32_t * num, uint32_t * den)
{
    uint8_t v[8];
    if (e->type == 5 && grn_tag_read(t, e, v, 8) == 8) { *num = grn_le32(v); *den = grn_le32(v + 4); }
}

static inline void grn_tag_ascii(struct grn_tiff_rd * t, const struct grn_tag * e, char * out, int size)
{
    memset(out, 0, size);
    if (e->type == 2) grn_tag_read(t, e, out, size - 1);
}

/* Canon ColorData: offsets (in int16 units) of as-shot WB, black and specular white;
 * count identifies the version table, only the first avail entries are in cd */
static inline void grn_colordata(struct grn_cr2 * c, const uint8_t * cd, int count, int avail)
{
    int ver = (int16_t) grn_le16(cd);
    int wb = 0x3f, bl = 0, wl = 0;

    if (count == 692 || count == 702 || count == 1227 || count == 1250 ||
        count == 1251 || count == 1337 || count == 1338 || count == 1346)
    {   /* ColorData4: 50D, 5D2, 7D, 550D, 60D ... */
        if (ver == 4 || ver == 5)      { bl = 0x2b4; wl = 0x2b9; }
        else if (ver == 6 || ver == 7) { bl = 0x2cb; wl = 0x2d0; }
        else if (ver == 9)             { bl = 0x2cf; wl = 0x2d4; }
    }
    else if (count == 1273 || count == 1275)
    {   /* ColorData6: 600D, 1200D */
        bl = 0x1df; wl = 0x1e4;
    }
    else if (count == 1312 || count == 1313 || count == 1316 || count == 1506)
    {   /* ColorData7: 5D3, 6D, 70D, 700D, 7D2, 750D, 760D */
        if (ver == 10)      { bl = 0x1f8; wl = 0x1fd; }
        else if (ver == 11) { bl = 0x2d8; wl = 0x2dd; }
    }
    else if (count == 1560 || count == 1592 || count == 1353 || count == 1602)
    {   /* ColorData8: 5DS, 80D, 1300D, 200D/77D/800D/6D2 */
        if (ver == 14) { bl = 0x22c; wl = 0x231; }
        else           { bl = 0x30a; wl = 0x30f; }
    }
    else return;

    if (avail >= wb + 4)
        for (int i = 0; i < 4; i++) c->wb[i] = (int16_t) grn_le16(cd + 2 * (wb + i));

    if (bl && wl && avail > wl)
    {
        int b = 0;
        for (int i = 0; i < 4; i++) b += grn_le16(cd + 2 * (bl + i));
        b = (b + 2) / 4;
        int w = grn_le16(cd + 2 * wl);
        /* sanity: 14-bit data, black well below white */
        if (b >= 64 && b < 4096 && w > b + 1024 && w <= 16383) { c->black = b; c->white = w; }
    }
}

static inline int grn_cr2_makernote(struct grn_cr2 * c, struct grn_tiff_rd * t, uint32_t off)
{
    struct grn_tag e[64];
    int n = grn_tiff_ifd(t, off, e, 64);
    if (n < 0) return n;

    for (int i = 0; i < n; i++)
    {
        if (e[i].tag == 0x00E0 && e[i].type == 3 && e[i].count >= 9)
        {
            uint8_t v[18];
            if (grn_tag_read(t, &e[i], v, 18) != 18) return GRN_CR2_ERR_READ;
            /* SensorInfo: 1 width, 2 height, 5 left, 6 top, 7 right, 8 bottom (inclusive) */
            c->x1 = grn_le16(v + 10); c->y1 = grn_le16(v + 12);
            c->x2 = grn_le16(v + 14) + 1; c->y2 = grn_le16(v + 16) + 1;
        }
        else if (e[i].tag == 0x4001 && e[i].type == 3 && e[i].count > 0x40 && e[i].count <= 4096)
        {
            /* only read what we need (white level is the furthest field) */
            int need = e[i].count;
            if (need > 0x320) need = 0x320;
            if (2 * need > t->buf_size) return GRN_CR2_ERR_FORMAT;
            if (t->read(t->ctx, e[i].value, t->buf, 2 * need) != 2 * need) return GRN_CR2_ERR_READ;
            grn_colordata(c, t->buf, e[i].count, need);
        }
    }
    return 0;
}

/* ------------------------------------------------------------------ */
/* lossless JPEG header                                                */
/* ------------------------------------------------------------------ */

static inline int grn_huff_build(struct grn_huff * h, const uint8_t * counts, const uint8_t * vals, int nvals)
{
    memset(h, 0, sizeof(*h));
    memcpy(h->vals, vals, nvals);
    int code = 0, k = 0;
    for (int len = 1; len <= 16; len++)
    {
        int n = counts[len - 1];
        h->maxcode[len] = n ? code + n - 1 : -1;
        h->valoff[len] = k - code;
        for (int i = 0; i < n; i++, code++, k++)
        {
            if (len <= GRN_HUFF_LUT_BITS)
            {
                int shift = GRN_HUFF_LUT_BITS - len;
                for (int j = 0; j < (1 << shift); j++)
                    h->lut[(code << shift) | j] = (len << 8) | vals[k];
            }

            /* fast table: every value of the difference bits that follow the code */
            int ssss = vals[k];
            if (ssss < 16 && len + ssss <= GRN_FAST_BITS)
            {
                int shift = GRN_FAST_BITS - len - ssss;
                for (int x = 0; x < (1 << ssss); x++)
                {
                    int diff = x;
                    if (ssss && !(x & (1 << (ssss - 1)))) diff -= (1 << ssss) - 1;
                    int16_t e = (int16_t) ((diff * 16) | (len + ssss));
                    int base = (((code << ssss) | x) << shift);
                    for (int j = 0; j < (1 << shift); j++)
                        h->fast[base | j] = e;
                }
            }
        }
        if (code > (1 << len)) return GRN_CR2_ERR_JPEG;
        code <<= 1;
    }
    h->valid = 1;
    return 0;
}

/* parse SOF3 / DHT / SOS at data_off (buffer: scratch of t) */
static inline int grn_ljpeg_header(struct grn_cr2 * c, struct grn_tiff_rd * t)
{
    uint8_t * b = t->buf;
    uint32_t off = c->data_off;
    if (t->read(t->ctx, off, b, 2) != 2) return GRN_CR2_ERR_READ;
    if (b[0] != 0xFF || b[1] != 0xD8) return GRN_CR2_ERR_JPEG;
    off += 2;

    int sof_ids[4] = { 0 };
    int have_sof = 0;

    for (;;)
    {
        if (off + 4 > c->data_end) return GRN_CR2_ERR_JPEG;
        if (t->read(t->ctx, off, b, 4) != 4) return GRN_CR2_ERR_READ;
        if (b[0] != 0xFF) return GRN_CR2_ERR_JPEG;
        int marker = b[1];
        int len = (b[2] << 8) | b[3];
        if (len < 2) return GRN_CR2_ERR_JPEG;
        uint32_t seg = off + 4;
        int plen = len - 2;
        off += 2 + len;

        if (marker == 0xC3 || marker == 0xC4 || marker == 0xDA)
        {
            if (plen > t->buf_size) return GRN_CR2_ERR_JPEG;
            if (t->read(t->ctx, seg, b, plen) != plen) return GRN_CR2_ERR_READ;
        }

        if (marker == 0xC3)                         /* SOF3: lossless, Huffman */
        {
            if (plen < 6) return GRN_CR2_ERR_JPEG;
            c->bits = b[0];
            c->jh = (b[1] << 8) | b[2];
            c->jw = (b[3] << 8) | b[4];
            c->clrs = b[5];
            if (c->clrs < 1 || c->clrs > 4 || plen < 6 + 3 * c->clrs) return GRN_CR2_ERR_JPEG;
            for (int i = 0; i < c->clrs; i++)
            {
                sof_ids[i] = b[6 + 3 * i];
                if (b[7 + 3 * i] != 0x11) return GRN_CR2_ERR_JPEG;   /* subsampled = sRAW/mRAW */
            }
            have_sof = 1;
        }
        else if (marker == 0xC4)                    /* DHT, possibly several tables */
        {
            int p = 0;
            while (p + 17 <= plen)
            {
                int tc = b[p] >> 4, th = b[p] & 15;
                int nv = 0;
                for (int i = 0; i < 16; i++) nv += b[p + 1 + i];
                if (tc != 0 || th > 3 || nv > 256 || p + 17 + nv > plen) return GRN_CR2_ERR_JPEG;
                int r = grn_huff_build(&c->huff[th], b + p + 1, b + p + 17, nv);
                if (r) return r;
                p += 17 + nv;
            }
        }
        else if (marker == 0xDA)                    /* SOS */
        {
            if (!have_sof) return GRN_CR2_ERR_JPEG;
            int ns = b[0];
            if (ns != c->clrs || plen < 1 + 2 * ns + 3) return GRN_CR2_ERR_JPEG;
            for (int i = 0; i < ns; i++)
            {
                int id = b[1 + 2 * i], td = b[2 + 2 * i] >> 4;
                if (id != sof_ids[i] || td > 3 || !c->huff[td].valid) return GRN_CR2_ERR_JPEG;
                c->comp_tbl[i] = td;
            }
            c->psv = b[1 + 2 * ns];
            if (c->psv != 1 || (b[3 + 2 * ns] & 15) != 0) return GRN_CR2_ERR_JPEG;
            c->scan_off = off;
            return 0;
        }
        else if (marker == 0xDD)                    /* restart intervals: not used by Canon */
            return GRN_CR2_ERR_JPEG;
    }
}

/* parse everything needed to decode and to write the DNG; scratch >= 2 KB */
static inline int grn_cr2_parse(struct grn_cr2 * c, void * ctx, grn_read_fn read, uint8_t * scratch, int scratch_size)
{
    memset(c, 0, sizeof(*c));
    struct grn_tiff_rd t = { ctx, read, scratch, scratch_size };
    struct grn_tag e[64];
    uint8_t h[16];

    if (read(ctx, 0, h, 16) != 16) return GRN_CR2_ERR_READ;
    if (h[0] != 'I' || h[1] != 'I' || grn_le16(h + 2) != 42 || h[8] != 'C' || h[9] != 'R')
        return GRN_CR2_ERR_FORMAT;
    uint32_t ifd0 = grn_le32(h + 4), raw_ifd = grn_le32(h + 12);

    /* IFD0 */
    uint32_t exif = 0;
    int n = grn_tiff_ifd(&t, ifd0, e, 64);
    if (n < 0) return n;
    for (int i = 0; i < n; i++)
    {
        switch (e[i].tag)
        {
            case 0x0110: grn_tag_ascii(&t, &e[i], c->model, sizeof(c->model)); break;
            case 0x0112: c->orientation = grn_tag_uint(&e[i]); break;
            case 0x0132: grn_tag_ascii(&t, &e[i], c->datetime, sizeof(c->datetime)); break;
            case 0x8769: exif = e[i].value; break;
        }
    }

    /* Exif IFD */
    uint32_t mnote = 0;
    if (exif)
    {
        n = grn_tiff_ifd(&t, exif, e, 64);
        if (n < 0) return n;
        for (int i = 0; i < n; i++)
        {
            switch (e[i].tag)
            {
                case 0x829A: grn_tag_rational(&t, &e[i], &c->exp_num, &c->exp_den); break;
                case 0x829D: grn_tag_rational(&t, &e[i], &c->fnum_num, &c->fnum_den); break;
                case 0x8827: c->iso = grn_tag_uint(&e[i]); break;
                case 0x9003: grn_tag_ascii(&t, &e[i], c->datetime, sizeof(c->datetime)); break;
                case 0x920A: grn_tag_rational(&t, &e[i], &c->focal_num, &c->focal_den); break;
                case 0x927C: mnote = e[i].value; break;
            }
        }
    }
    if (mnote)
    {
        int r = grn_cr2_makernote(c, &t, mnote);
        if (r) return r;
    }

    /* raw IFD (IFD3) */
    int cfa = 1;
    n = grn_tiff_ifd(&t, raw_ifd, e, 64);
    if (n < 0) return n;
    for (int i = 0; i < n; i++)
    {
        switch (e[i].tag)
        {
            case 0x0111: c->data_off = grn_tag_uint(&e[i]); break;
            case 0x0117: c->data_end = grn_tag_uint(&e[i]); break;
            case 0xC5E0: cfa = grn_tag_uint(&e[i]); break;
            case 0xC640:
            {
                uint8_t v[6];
                if (e[i].count != 3 || grn_tag_read(&t, &e[i], v, 6) != 6) return GRN_CR2_ERR_FORMAT;
                c->slice_n = grn_le16(v); c->slice_w = grn_le16(v + 2); c->slice_last = grn_le16(v + 4);
                break;
            }
        }
    }
    if (!c->data_off || !c->data_end) return GRN_CR2_ERR_FORMAT;
    c->data_end += c->data_off;

    static const uint8_t cfa_tab[4][4] = { { 0, 1, 1, 2 }, { 2, 1, 1, 0 }, { 1, 2, 0, 1 }, { 1, 0, 2, 1 } };
    memcpy(c->cfa, cfa_tab[(cfa >= 1 && cfa <= 4) ? cfa - 1 : 0], 4);

    int r = grn_ljpeg_header(c, &t);
    if (r) return r;

    /* sensor frame size from the JPEG geometry (dcraw: apply_tiff) */
    c->width = c->jw * c->clrs;
    c->height = c->jh;
    if (c->width > 4 * c->height && !(c->clrs & 1)) { c->width /= 2; c->height *= 2; }

    /* slices */
    if (c->slice_n)
    {
        c->nslices = c->slice_n + 1;
        if (c->nslices > GRN_CR2_MAX_SLICES || c->slice_n * c->slice_w + c->slice_last != c->width)
            return GRN_CR2_ERR_GEOMETRY;
        for (int k = 0; k < c->nslices; k++)
        {
            c->slice_x[k] = k * c->slice_w;
            c->slice_wd[k] = k < c->slice_n ? c->slice_w : c->slice_last;
        }
    }
    else
    {
        c->nslices = 1;
        c->slice_x[0] = 0;
        c->slice_wd[0] = c->width;
    }

    /* no SensorInfo: whole frame is "active" */
    if (c->x2 <= c->x1 || c->y2 <= c->y1 || c->x2 > c->width || c->y2 > c->height)
    {
        c->x1 = c->y1 = 0; c->x2 = c->width; c->y2 = c->height;
    }
    if (c->bits > 14 || c->bits < 8) return GRN_CR2_ERR_JPEG;
    return 0;
}

/* ------------------------------------------------------------------ */
/* lossless JPEG decoder (resumable)                                   */
/* ------------------------------------------------------------------ */

struct grn_ljstate
{
    uint32_t pos;                   /* file offset of the next unread byte */
    uint32_t bitbuf;                /* left aligned */
    int bitcnt, marker;
    int comp, col;                  /* position inside the current JPEG row */
    int vpred[4], left[4];
};

struct grn_ljdec
{
    const struct grn_cr2 * c;
    void * ctx;
    grn_read_fn read;
    uint8_t * buf;                  /* cached input chunk */
    int buf_size;
    uint32_t buf_off;               /* file offset of buf[0] */
    int buf_len, buf_pos;
    struct grn_ljstate s;
    int error;
};

static inline void grn_lj_init(struct grn_ljdec * d, const struct grn_cr2 * c, void * ctx, grn_read_fn read, uint8_t * buf, int buf_size)
{
    memset(d, 0, sizeof(*d));
    d->c = c; d->ctx = ctx; d->read = read; d->buf = buf; d->buf_size = buf_size;
    d->s.pos = c->scan_off;
    d->buf_off = c->scan_off;
    for (int i = 0; i < 4; i++) d->s.vpred[i] = 1 << (c->bits - 1);
}

static inline void grn_lj_save(const struct grn_ljdec * d, struct grn_ljstate * st)
{
    *st = d->s;
    st->pos = d->buf_off + d->buf_pos;
}

static inline void grn_lj_restore(struct grn_ljdec * d, const struct grn_ljstate * st)
{
    d->s = *st;
    d->buf_off = st->pos;
    d->buf_len = d->buf_pos = 0;
}

static inline int grn_lj_byte(struct grn_ljdec * d)
{
    if (d->buf_pos >= d->buf_len)
    {
        uint32_t off = d->buf_off + d->buf_len;
        int n = (int) (d->c->data_end - off);
        if (n <= 0) return -1;
        if (n > d->buf_size) n = d->buf_size;
        if (d->read(d->ctx, off, d->buf, n) != n) { d->error = GRN_CR2_ERR_READ; return -1; }
        d->buf_off = off; d->buf_len = n; d->buf_pos = 0;
    }
    return d->buf[d->buf_pos++];
}

/* next entropy-coded byte, slow path: chunk refill and 0xFF handling.
 * 0xFF 0x00 = stuffed 0xFF; any other 0xFF xx is a marker (EOI): from then on, feed zeros */
static inline int grn_lj_next(struct grn_ljdec * d, int * marker)
{
    if (*marker) return 0;
    int b = grn_lj_byte(d);
    if (b < 0 || (b == 0xFF && grn_lj_byte(d) != 0)) { *marker = 1; return 0; }
    return b;
}

/*
 * advance n samples. full: compute every value (out may be NULL);
 * !full: only follow the bit stream and the first-column predictors, which is
 * enough to resume full decoding at the start of a JPEG row (pass 1).
 * State lives in locals: ML builds with -fno-strict-aliasing, so every store
 * to out[] would otherwise force reloading the whole state from *d.
 */
static inline int grn_lj_run(struct grn_ljdec * d, uint16_t * out, int n, const int full)
{
    const struct grn_cr2 * c = d->c;
    const int clrs = c->clrs, jw = c->jw;
    const unsigned maxv = (1u << c->bits) - 1;
    const struct grn_huff * h[4];
    for (int k = 0; k < clrs; k++) h[k] = &c->huff[c->comp_tbl[k]];

    uint32_t bitbuf = d->s.bitbuf;
    int bitcnt = d->s.bitcnt, marker = d->s.marker;
    int comp = d->s.comp, col = d->s.col;
    int left[4], vpred[4];
    memcpy(left, d->s.left, sizeof(left));
    memcpy(vpred, d->s.vpred, sizeof(vpred));
    int err = 0;

    /* keep >= 25 bits in bitbuf (left aligned); fast path: plain byte from the chunk */
    #define GRN_FILL() \
        while (bitcnt <= 24) \
        { \
            int b_; \
            if (!marker && d->buf_pos < d->buf_len && d->buf[d->buf_pos] != 0xFF) b_ = d->buf[d->buf_pos++]; \
            else b_ = grn_lj_next(d, &marker); \
            bitbuf |= (uint32_t) b_ << (24 - bitcnt); \
            bitcnt += 8; \
        }

    for (int i = 0; i < n; i++)
    {
        GRN_FILL();
        const struct grn_huff * hh = h[comp];
        int diff;

        /* common case: code and difference bits in one lookup */
        int f = hh->fast[bitbuf >> (32 - GRN_FAST_BITS)];
        if (f)
        {
            int nb = f & 15;
            bitbuf <<= nb; bitcnt -= nb;
            diff = f >> 4;
            goto have_diff;
        }

        /* Huffman: length of the difference (SSSS) */
        int e = hh->lut[bitbuf >> (32 - GRN_HUFF_LUT_BITS)];
        int l, len;
        if (e)
        {
            l = e >> 8;
            len = e & 0xFF;
        }
        else
        {
            for (l = GRN_HUFF_LUT_BITS + 1; l <= 16; l++)
                if ((int32_t) (bitbuf >> (32 - l)) <= hh->maxcode[l]) break;
            if (l > 16) { err = GRN_CR2_ERR_DATA; break; }
            len = hh->vals[(bitbuf >> (32 - l)) + hh->valoff[l]];
        }
        bitbuf <<= l; bitcnt -= l;

        if (!full && col)
        {
            /* skip the difference bits; left[] goes stale until the next row start */
            if (len > 16) { err = GRN_CR2_ERR_DATA; break; }
            if (len && len < 16)
            {
                GRN_FILL();
                bitbuf <<= len; bitcnt -= len;
            }
            if (++comp == clrs)
            {
                comp = 0;
                if (++col == jw) col = 0;
            }
            continue;
        }

        /* difference bits */
        diff = 0;
        if (len >= 16)
        {
            if (len > 16) { err = GRN_CR2_ERR_DATA; break; }
            diff = -32768;
        }
        else if (len)
        {
            GRN_FILL();
            diff = bitbuf >> (32 - len);
            bitbuf <<= len; bitcnt -= len;
            if (!(diff & (1 << (len - 1)))) diff -= (1 << len) - 1;
        }

    have_diff:
        if (!full && col)
        {
            if (++comp == clrs)
            {
                comp = 0;
                if (++col == jw) col = 0;
            }
            continue;
        }

        /* predictor 1 (left); first column: same component of the previous row start */
        int v = col ? left[comp] + diff : (vpred[comp] += diff);
        left[comp] = v;
        if ((unsigned) v > maxv) { err = GRN_CR2_ERR_DATA; break; }
        if (out) out[i] = v;
        if (++comp == clrs)
        {
            comp = 0;
            if (++col == jw) col = 0;
        }
    }
    #undef GRN_FILL

    d->s.bitbuf = bitbuf; d->s.bitcnt = bitcnt; d->s.marker = marker;
    d->s.comp = comp; d->s.col = col;
    memcpy(d->s.left, left, sizeof(left));
    memcpy(d->s.vpred, vpred, sizeof(vpred));
    return d->error ? d->error : err;
}

static inline int grn_lj_decode(struct grn_ljdec * d, uint16_t * out, int n) { return grn_lj_run(d, out, n, 1); }
static inline int grn_lj_skip(struct grn_ljdec * d, int n)                   { return grn_lj_run(d, 0, n, 0); }

/* ------------------------------------------------------------------ */
/* CR2 -> DNG                                                          */
/* ------------------------------------------------------------------ */

struct grn_cr2_job
{
    struct grn_cr2 * c;
    int strength, mode;                   /* strength 0 = passthrough */
    int plate_w, plate_h, plate_black, plate_white;
    struct grn_blend_params bp;           /* set up after pass 1 (black may be measured there) */
    volatile int progress;                /* percent, for the menu */

    /* buffers, caller-allocated (DMA = may be uncacheable, only touched by memcpy) */
    uint8_t * in[GRN_CR2_MAX_SLICES];     /* cached input chunks, one per slice */
    int in_size;
    uint16_t * row;                       /* width */
    uint16_t * plate_row;                 /* optional cached plate_w buffer (plate_strip in DMA memory);
                                           * NULL: blend straight from plate_strip (cached strip only) */
    uint8_t * packed_row;                 /* optional cached pitch buffer, then memcpy to out (DMA);
                                           * NULL: pack straight into out (cached out only) */
    uint8_t * out;                        /* strip_rows * pitch */
    uint8_t * plate_strip;                /* plate_rows * plate_w * 2, DMA */
    int strip_rows, plate_rows;

    /* I/O */
    void * ctx;
    void * slice_ctx[GRN_CR2_MAX_SLICES]; /* optional ctx per slice decoder for read_cr2 (own file handle) */
    grn_read_fn read_cr2;
    int  (*read_plate)(void * ctx, void * buf, int bytes);
    int  (*write_out) (void * ctx, const void * buf, int bytes);
    void (*yield)     (void * ctx);
    int  (*now_ms)    (void);             /* optional, for t_index / t_total */
    int t_index, t_total;
};

/* local string helpers: strncmp/strncpy are not exported by every ML core */
static inline void grn_strcpy(char * dst, const char * src, int size)
{
    int i = 0;
    for (; i < size - 1 && src[i]; i++) dst[i] = src[i];
    dst[i] = 0;
}

/* model equals name, or is name + " suffix" ("Canon EOS 5DS R" matches "Canon EOS 5DS") */
static inline int grn_model_is(const char * model, const char * name)
{
    while (*name && *model == *name) { model++; name++; }
    return !*name && (!*model || *model == ' ');
}

/* DNG metadata from the CR2; matrix/white fallbacks from a small model table */
struct grn_cam_tab { const char * model; int white; int16_t cm[9]; };

static inline void grn_cr2_dng_info(const struct grn_cr2 * c, struct grn_dng_info * di)
{
    /* Adobe D65 matrices (LibRaw colordata.cpp) */
    static const struct grn_cam_tab tab[] = {
        { "Canon EOS 750D",        15360, { 6362,-823,-847,-4426,12109,2616,-743,1857,5635 } },
        { "Canon EOS 760D",        15360, { 6362,-823,-847,-4426,12109,2616,-743,1857,5635 } },
        { "Canon EOS 7D Mark II",  13584, { 7268,-1082,-969,-4186,11839,2663,-825,2029,5839 } },
        { "Canon EOS 80D",         0,     { 7457,-671,-937,-4849,12495,2643,-1213,2354,5492 } },
        { "Canon EOS 5DS",         15510, { 6250,-711,-808,-5153,12794,2636,-1249,2198,5610 } },
        { "Canon EOS 1300D",       14274, { 6939,-1016,-866,-4428,12473,2177,-1175,2178,6162 } },
        { "Canon EOS 200D",        0,     { 7377,-742,-998,-4235,11981,2549,-673,1918,5538 } },
        { "Canon EOS 77D",         0,     { 7377,-742,-998,-4235,11981,2549,-673,1918,5538 } },
        { "Canon EOS 800D",        0,     { 6970,-512,-968,-4425,12161,2553,-739,1982,5601 } },
        { "Canon EOS 6D Mark II",  14558, { 6875,-970,-932,-4691,12459,2501,-874,1953,5809 } },
        { "Canon EOS 550D",        15831, { 6941,-1164,-857,-3825,11597,2534,-416,1540,6039 } },
    };
    /* unknown model: 750D matrix as an approximation (similar Canon CFA) */
    const struct grn_cam_tab * m = &tab[0];
    for (unsigned i = 0; i < sizeof(tab) / sizeof(tab[0]); i++)
        if (grn_model_is(c->model, tab[i].model)) { m = &tab[i]; break; }

    memset(di, 0, sizeof(*di));
    di->width = c->width;
    di->height = c->height;
    di->pitch = c->width * 14 / 8;
    di->black = c->black;
    di->white = c->white ? c->white : (m->white ? m->white : 16383);
    for (int k = 0; k < 9; k++) { di->color_matrix1[2*k] = m->cm[k]; di->color_matrix1[2*k+1] = 10000; }
    di->active[0] = c->y1; di->active[1] = c->x1; di->active[2] = c->y2; di->active[3] = c->x2;
    /* DNG CFAPattern starts at the active area origin */
    for (int y = 0; y < 2; y++)
        for (int x = 0; x < 2; x++)
            di->cfa[2*y + x] = c->cfa[2 * ((y + c->y1) & 1) + ((x + c->x1) & 1)];
    grn_strcpy(di->model, c->model[0] ? c->model : "Canon EOS", sizeof(di->model));
    grn_strcpy(di->datetime, c->datetime, sizeof(di->datetime));
    di->orientation = c->orientation;
    di->iso = c->iso;
    di->exp_num = c->exp_num; di->exp_den = c->exp_den;
    di->fnum_num = c->fnum_num; di->fnum_den = c->fnum_den;
    di->focal_num = c->focal_num; di->focal_den = c->focal_den;
    if (c->wb[0] > 0 && c->wb[1] > 0 && c->wb[2] > 0 && c->wb[3] > 0)
    {
        int g = (c->wb[1] + c->wb[2]) / 2;
        di->neutral[0] = g; di->neutral[1] = c->wb[0];
        di->neutral[2] = 1; di->neutral[3] = 1;
        di->neutral[4] = g; di->neutral[5] = c->wb[3];
    }
}

/* plain division: keeps GCC from fusing a / b * b into __aeabi_uidivmod,
 * which older ML cores may not export (__aeabi_uidiv they all do) */
static uint32_t __attribute__((noinline)) grn_udiv(uint32_t a, uint32_t b) { return a / b; }

/*
 * Pass 1: find the decoder state at each slice start. Only the bit stream and
 * the first-column predictors are followed (grn_lj_skip), plus a full decode
 * of the JPEG row part before a slice start (left predictors). If the
 * makernote had no black level, slice 0 is decoded fully to measure it on the
 * masked columns left of the active area.
 */
static inline int grn_cr2_index(struct grn_cr2_job * j, struct grn_ljstate * st)
{
    struct grn_cr2 * c = j->c;
    const int H = c->height;
    struct grn_ljdec d;
    grn_lj_init(&d, c, j->slice_ctx[0] ? j->slice_ctx[0] : j->ctx, j->read_cr2, j->in[0], j->in_size);
    grn_lj_save(&d, &st[0]);

    const uint32_t rowlen = c->jw * c->clrs;                            /* samples per JPEG row */
    const uint32_t last = (uint32_t) c->slice_x[c->nslices - 1] * H;    /* start of the last slice */
    uint32_t pos = 0;
    int r;

    /* masked columns; stay well left of the JPEG crop, the first columns right
     * of x1/2 can already hold image data (750D: dcraw starts the image at 72, x1 = 84) */
    const int w0 = c->slice_wd[0];
    const int ob_x0 = c->x1 / 8, ob_x1 = c->x1 / 2;
    if (!c->black && ob_x1 - ob_x0 >= 8 && ob_x1 <= w0)
    {
        uint32_t ob_sum = 0, ob_n = 0;
        for (int y = 0; y < H; y++)
        {
            if ((r = grn_lj_decode(&d, j->row, w0))) return r;
            if (y >= c->y1 && y < c->y2)
            {
                for (int x = ob_x0; x < ob_x1; x++) ob_sum += j->row[x];
                ob_n += ob_x1 - ob_x0;
            }
            if ((y & 63) == 63 && j->yield) j->yield(j->ctx);
        }
        pos = (uint32_t) w0 * H;
        if (ob_n) c->black = (ob_sum + ob_n / 2) / ob_n;
    }
    if (!c->black) return GRN_CR2_ERR_FORMAT;

    for (int k = 1; k < c->nslices; k++)
    {
        uint32_t b = (uint32_t) c->slice_x[k] * H;      /* linear sample index of the slice start */
        uint32_t row0 = grn_udiv(b, rowlen) * rowlen;   /* start of its JPEG row */
        while (pos < row0)
        {
            uint32_t n = row0 - pos;
            if (n > 64 * rowlen) n = 64 * rowlen;
            if ((r = grn_lj_skip(&d, n))) return r;
            pos += n;
            j->progress = 30 * (pos / 1024) / (last / 1024 + 1);
            if (j->yield) j->yield(j->ctx);
        }
        if (b > pos)
        {
            if ((r = grn_lj_decode(&d, 0, b - pos))) return r;
            pos = b;
        }
        grn_lj_save(&d, &st[k]);
    }
    return 0;
}

/* header + pass 1 + pass 2; returns 0 or GRN_CR2_ERR_* */
static inline int grn_cr2_to_dng(struct grn_cr2_job * j, uint8_t * hdr, int hdr_size)
{
    struct grn_cr2 * c = j->c;
    const int W = c->width, H = c->height;
    const int pitch = W * 14 / 8;
    const int plate_row_bytes = j->plate_w * 2;
    const int blend = j->strength > 0;

    if (W & 7) return GRN_CR2_ERR_GEOMETRY;
    if (blend && (j->plate_w != c->x2 - c->x1 || j->plate_h != c->y2 - c->y1))
        return GRN_CR2_ERR_GEOMETRY;

    j->progress = 0;
    const int t0 = j->now_ms ? j->now_ms() : 0;
    struct grn_ljstate st[GRN_CR2_MAX_SLICES];
    int r = grn_cr2_index(j, st);
    if (r) return r;
    if (j->now_ms) j->t_index = j->now_ms() - t0;

    /* header */
    struct grn_dng_info di;
    grn_cr2_dng_info(c, &di);
    if (blend && grn_blend_setup(&j->bp, j->mode, j->strength, di.black, di.white, j->plate_black, j->plate_white))
        return GRN_CR2_ERR_PLATE;
    int hs = grn_dng_build_header(hdr, hdr_size, &di);
    if (hs < 0) return GRN_CR2_ERR_FORMAT;
    if (j->write_out(j->ctx, hdr, hs) != hs) return GRN_CR2_ERR_WRITE;

    /* pass 2: one decoder per slice */
    struct grn_ljdec d[GRN_CR2_MAX_SLICES];
    for (int k = 0; k < c->nslices; k++)
    {
        grn_lj_init(&d[k], c, j->slice_ctx[k] ? j->slice_ctx[k] : j->ctx, j->read_cr2, j->in[k], j->in_size);
        grn_lj_restore(&d[k], &st[k]);
    }

    int plate_start = 0, plate_rows = 0;
    int n_out = 0;

    for (int y = 0; y < H; y++)
    {
        for (int k = 0; k < c->nslices; k++)
        {
            r = grn_lj_decode(&d[k], j->row + c->slice_x[k], c->slice_wd[k]);
            if (r) return r;
        }

        if (blend && y >= c->y1 && y < c->y2)
        {
            int py = y - c->y1;
            if (py >= plate_start + plate_rows)
            {
                plate_start += plate_rows;
                int m = j->plate_h - plate_start;
                if (m > j->plate_rows) m = j->plate_rows;
                if (j->read_plate(j->ctx, j->plate_strip, m * plate_row_bytes) != m * plate_row_bytes)
                    return GRN_CR2_ERR_PLATE;
                plate_rows = m;
            }
            const uint16_t * pr = (const uint16_t *) (j->plate_strip + (py - plate_start) * plate_row_bytes);
            if (j->plate_row)
            {
                memcpy(j->plate_row, pr, plate_row_bytes);
                pr = j->plate_row;
            }
            grn_blend_span(&j->bp, j->row, c->x1, pr, j->plate_w);
        }

        uint8_t * dst = j->out + n_out * pitch;
        uint8_t * pk = j->packed_row ? j->packed_row : dst;
        grn_pack_row_dng(j->row, pk, W);
        if (pk != dst) memcpy(dst, pk, pitch);

        if (++n_out == j->strip_rows || y == H - 1)
        {
            if (j->write_out(j->ctx, j->out, n_out * pitch) != n_out * pitch)
                return GRN_CR2_ERR_WRITE;
            n_out = 0;
            j->progress = 30 + 70 * (y + 1) / H;
            if (j->yield) j->yield(j->ctx);
        }
    }
    if (j->now_ms) j->t_total = j->now_ms() - t0;
    return 0;
}

#endif
