#!/usr/bin/env python3
"""
ESP32-P4 camera UDP frame recorder (Linux / headless)
Reassembles chunked MJPEG frames (one JPEG per video frame) and saves them
directly as .jpg files — the firmware already did the compression in hardware,
so no decode/debayer is needed here.

Usage:
    python3 recorder_nas.py [--port 12345] [--outdir /tmp/frames]

The newest frame is also written to <outdir>/latest.jpg so you can
monitor it with any HTTP file-serving you already have on the NAS.
"""

import argparse
import os
import socket
import struct

HDR_FMT  = "<IHHII"   # must match frame_chunk_hdr_t in cam_csi_eth_stream_main.c
HDR_SIZE = struct.calcsize(HDR_FMT)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port",   type=int, default=12345)
    ap.add_argument("--outdir", default="/tmp/frames")
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("0.0.0.0", args.port))
    print(f"Listening on UDP :{args.port}  saving JPEGs to {args.outdir}/")

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
            jpeg = b"".join(chunk_buffer[frame_num][i] for i in range(total_chunks))
            del chunk_buffer[frame_num]

            if len(jpeg) != frame_bytes:
                print(f"Frame {frame_num}: size mismatch {len(jpeg)} != {frame_bytes}, skipping")
                continue

            path = os.path.join(args.outdir, f"frame_{frame_num:06d}.jpg")
            with open(path, "wb") as f:
                f.write(jpeg)
            # Overwrite latest.jpg atomically so a web viewer always sees a complete file
            latest = os.path.join(args.outdir, "latest.jpg")
            tmp = latest + ".tmp"
            with open(tmp, "wb") as f:
                f.write(jpeg)
            os.replace(tmp, latest)
            print(f"Saved frame {frame_num:6d}  {len(jpeg):,} B  -> {path}")

if __name__ == "__main__":
    main()
