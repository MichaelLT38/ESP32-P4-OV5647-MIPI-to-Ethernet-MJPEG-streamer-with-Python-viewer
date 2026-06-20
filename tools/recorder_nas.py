#!/usr/bin/env python3
"""
ESP32-P4 camera UDP frame recorder (Linux / headless)
Reassembles chunked RAW8 Bayer frames, debayers them to RGB, and saves PNGs.

Usage:
    pip3 install numpy Pillow                 (minimal: numpy 2x2 debayer fallback)
    pip3 install numpy Pillow opencv-python   (optional: full-res OpenCV debayer)

    python3 recorder_nas.py [--port 12345] [--width 800] [--height 640] \
                            [--bayer BGGR] [--outdir /tmp/frames]

The newest frame is also written to <outdir>/latest.png so you can
monitor it with any HTTP file-serving you already have on the NAS.
"""

import argparse
import os
import socket
import struct
import numpy as np
from PIL import Image

# Optional high-quality debayer via OpenCV; falls back to numpy if unavailable.
try:
    import cv2
    _CV2_BAYER = {
        "BGGR": cv2.COLOR_BayerBGGR2RGB,
        "RGGB": cv2.COLOR_BayerRGGB2RGB,
        "GBRG": cv2.COLOR_BayerGBRG2RGB,
        "GRBG": cv2.COLOR_BayerGRBG2RGB,
    }
except ImportError:
    cv2 = None
    _CV2_BAYER = {}

HDR_FMT  = "<IHHII"   # must match frame_chunk_hdr_t in cam_csi_eth_stream_main.c
HDR_SIZE = struct.calcsize(HDR_FMT)

def raw8_to_pil(data: bytes, width: int, height: int, bayer: str) -> Image.Image:
    """Debayer RAW8 Bayer data to an RGB PIL image.

    Uses OpenCV for a full-resolution result when available, otherwise falls
    back to a dependency-free numpy 2x2 block debayer (half resolution).
    """
    bayer_plane = np.frombuffer(data, dtype=np.uint8).reshape(height, width)

    if cv2 is not None:
        rgb = cv2.cvtColor(bayer_plane, _CV2_BAYER[bayer])
        return Image.fromarray(rgb, "RGB")

    # numpy fallback: combine each 2x2 Bayer block into one RGB pixel.
    # Pixel positions within a 2x2 block for each supported pattern:
    #   key = (row0col0, row0col1, row1col0, row1col1)
    layout = {
        "BGGR": ("B", "G", "G", "R"),
        "RGGB": ("R", "G", "G", "B"),
        "GBRG": ("G", "B", "R", "G"),
        "GRBG": ("G", "R", "B", "G"),
    }[bayer]

    c00 = bayer_plane[0::2, 0::2]
    c01 = bayer_plane[0::2, 1::2]
    c10 = bayer_plane[1::2, 0::2]
    c11 = bayer_plane[1::2, 1::2]
    cells = {"00": c00, "01": c01, "10": c10, "11": c11}
    pos_keys = ["00", "01", "10", "11"]

    r = b = None
    g_sum = None
    g_count = 0
    for color, key in zip(layout, pos_keys):
        plane = cells[key].astype(np.uint16)
        if color == "R":
            r = plane
        elif color == "B":
            b = plane
        else:  # two green samples — average them
            g_sum = plane if g_sum is None else (g_sum + plane)
            g_count += 1

    g = (g_sum // max(g_count, 1)).astype(np.uint8)
    rgb = np.stack([r.astype(np.uint8), g, b.astype(np.uint8)], axis=-1)
    return Image.fromarray(rgb, "RGB")

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port",   type=int, default=12345)
    ap.add_argument("--width",  type=int, default=800)
    ap.add_argument("--height", type=int, default=640)
    ap.add_argument("--bayer",  default="BGGR",
                    choices=["BGGR", "RGGB", "GBRG", "GRBG"],
                    help="Bayer pattern (OV5647 is BGGR)")
    ap.add_argument("--outdir", default="/tmp/frames")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)
    expected = args.width * args.height  # RAW8 = 1 byte/pixel

    if cv2 is None:
        print("OpenCV not found — using numpy 2x2 debayer (half resolution). "
              "Install opencv-python for full-res output.")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    print(f"Listening on UDP :{args.port}  saving to {args.outdir}/")

    chunk_buffer: dict = {}

    while True:
        data, addr = sock.recvfrom(HDR_SIZE + 64 * 1024)
        if len(data) < HDR_SIZE:
            continue

        frame_num, chunk_idx, total_chunks, frame_bytes, chunk_bytes = \
            struct.unpack_from(HDR_FMT, data, 0)
        payload = data[HDR_SIZE : HDR_SIZE + chunk_bytes]

        if frame_num not in chunk_buffer:
            chunk_buffer[frame_num] = {}
        chunk_buffer[frame_num][chunk_idx] = payload

        for old in [k for k in chunk_buffer if k < frame_num - 4]:
            del chunk_buffer[old]

        if len(chunk_buffer[frame_num]) == total_chunks:
            raw = b"".join(chunk_buffer[frame_num][i] for i in range(total_chunks))
            del chunk_buffer[frame_num]

            if len(raw) != expected:
                print(f"Frame {frame_num}: size mismatch {len(raw)} != {expected}, skipping")
                continue

            img = raw8_to_pil(raw, args.width, args.height, args.bayer)
            path = os.path.join(args.outdir, f"frame_{frame_num:06d}.png")
            img.save(path)
            # Overwrite latest.png atomically so a web viewer always sees a complete file
            latest = os.path.join(args.outdir, "latest.png")
            img.save(latest + ".tmp")
            os.replace(latest + ".tmp", latest)
            print(f"Saved frame {frame_num:6d}  {len(raw):,} B  -> {path}")

if __name__ == "__main__":
    main()
