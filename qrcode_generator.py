#!/usr/bin/env python3
"""
url_to_qr.py

Generate a QR code image from a given URL.
"""

import argparse
import sys

try:
    import qrcode
except ImportError:
    print("Missing dependency: install with `pip install qrcode[pil]`")
    sys.exit(1)


def generate_qr(url: str, output_file: str, box_size: int = 10, border: int = 4):
    """
    Generate and save a QR code for the given URL.

    :param url: The website URL to encode.
    :param output_file: Path to save the generated PNG file.
    :param box_size: Size of each box in pixels.
    :param border: Border width (boxes thick).
    """
    qr = qrcode.QRCode(
        version=None,  # automatic sizing
        error_correction=qrcode.constants.ERROR_CORRECT_M,
        box_size=box_size,
        border=border,
    )
    qr.add_data(url)
    qr.make(fit=True)

    img = qr.make_image(fill_color="black", back_color="white")
    img.save(output_file)
    print(f"QR code saved to {output_file}")


def main():
    parser = argparse.ArgumentParser(
        description="Generate a QR code image from a website URL."
    )
    parser.add_argument(
        "url",
        nargs="?",
        help="The URL to encode as a QR code. If omitted, you’ll be prompted interactively.",
    )
    parser.add_argument(
        "-o", "--output",
        default="qrcode.png",
        help="Output filename (default: qrcode.png)",
    )
    parser.add_argument(
        "--box-size",
        type=int,
        default=10,
        help="Pixel size of each QR code box (default: 10)",
    )
    parser.add_argument(
        "--border",
        type=int,
        default=4,
        help="Border thickness in boxes (default: 4)",
    )
    args = parser.parse_args()

    if not args.url:
        args.url = input("Enter the URL to encode: ").strip()
        if not args.url:
            print("Error: No URL provided.")
            sys.exit(1)

    generate_qr(args.url, args.output, box_size=args.box_size, border=args.border)


if __name__ == "__main__":
    main()

