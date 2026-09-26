#!/usr/bin/env python3
"""
make_grain_plate.py — Convert a scanned film-grain JPEG into a Magic Lantern
grain plate (.grn) for the grain_overlay module.

Usage:
    python make_grain_plate.py grain_scan.jpg --width 5760 --height 3840
    python make_grain_plate.py bw_negative_scan.jpg --width 5760 --height 3840 --invert

Common sensor active areas (verify on camera via the module's status line):
    5D3  5760x3840     5D2 5616x3744     6D  5472x3648
    700D 5184x3456     EOS M 5184x3456   5D4 6720x4480

The plate is grayscale, so Bayer phase does not matter.
"""
import argparse
import struct
import sys

import numpy as np
from PIL import Image, ImageFilter

MAGIC = b"GRN1"
VERSION = 1
SCALE14 = 16383  # 14-bit max


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("input", help="grain scan image (JPEG/TIFF/PNG)")
    ap.add_argument("-o", "--output", default="grain.grn")
    ap.add_argument("--width", type=int, required=True,
                    help="camera active-area width in pixels")
    ap.add_argument("--height", type=int, required=True,
                    help="camera active-area height in pixels")
    ap.add_argument("--invert", action="store_true",
                    help="set if the scan is a negative (dark grain on light base)")
    ap.add_argument("--black-pct", type=float, default=0.5,
                    help="percentile used as black point (default 0.5)")
    ap.add_argument("--white-pct", type=float, default=99.5,
                    help="percentile used as white point (default 99.5)")
    ap.add_argument("--preblur", type=float, default=0.6,
                    help="Gaussian radius (in scan pixels) before resizing; "
                         "prevents aliasing when downsampling (0 to disable)")
    ap.add_argument("--strength-scale", type=float, default=1.0,
                    help="premultiply plate brightness, e.g. 0.7 for subtler grain")
    args = ap.parse_args()

    im = Image.open(args.input)
    im = im.convert("L")          # grain must be neutral
    if args.invert:
        print("inverted")
        im = Image.eval(im, lambda v: 255 - v)
    if args.preblur > 0:
        im = im.filter(ImageFilter.GaussianBlur(args.preblur))
    if im.size != (args.width, args.height):
        im = im.resize((args.width, args.height), Image.LANCZOS)

    a = np.asarray(im, dtype=np.uint8).astype(np.float64)

    # auto-levels on the scan itself
    bp = np.percentile(a, args.black_pct)
    wp = np.percentile(a, args.white_pct)
    if wp <= bp:
        sys.exit("error: degenerate scan (white point == black point)")
    a = (a - bp) / (wp - bp)
    a = np.clip(a * args.strength_scale, 0.0, 1.0)
    a = np.round(a * SCALE14).astype("<u2")   # uint16 LE, 14-bit range

    header = struct.pack("<4sIIIII", MAGIC, VERSION,
                         args.width, args.height, 0, SCALE14)
    with open(args.output, "wb") as f:
        f.write(header)
        f.write(a.tobytes())

    print(f"wrote {args.output}: {args.width}x{args.height}, "
          f"{(24 + a.nbytes) / 1e6:.1f} MB "
          f"(black={bp:.1f}, white={wp:.1f} on 8-bit scale)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
