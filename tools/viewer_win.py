"""
ESP32-P4 camera UDP frame viewer (Windows)
Receives chunked MJPEG frames (one JPEG per video frame) and displays them live.

The firmware encodes each 1920x1080 frame to JPEG (hardware encoder, 4:2:0)
and streams the compressed bytes in 1400-byte UDP chunks. This viewer
reassembles the chunks and decodes the JPEG with OpenCV — no Bayer/debayer
or resolution handling is needed, the JPEG carries its own dimensions.

Usage:
    pip install opencv-python numpy
    python viewer_win.py [--port 12345]
"""

import argparse
import socket
import struct
import sys

def log(msg):
    print(msg, flush=True)

try:
    import numpy as np
    import cv2
except ImportError as e:
    print(f"ERROR: Missing dependency — {e}")
    print("Run:  pip install opencv-python numpy")
    sys.exit(1)

# Header layout must match frame_chunk_hdr_t in cam_csi_eth_stream_main.c
# uint32 frame_num | uint16 chunk_idx | uint16 total_chunks | uint32 frame_bytes | uint32 chunk_bytes
# frame_bytes is the size of the whole compressed JPEG for this frame.
HDR_FMT  = "<IHHII"
HDR_SIZE = struct.calcsize(HDR_FMT)   # 16 bytes

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", type=int, default=12345)
    args = ap.parse_args()

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", args.port))
        sock.settimeout(5.0)
    except OSError as e:
        log(f"[ERR]  Could not bind UDP port {args.port}: {e}")
        log("       Is another instance of this script already running?")
        sys.exit(1)

    log(f"[OK]  Socket bound on 0.0.0.0:{args.port}")
    log("[OK]  Expecting MJPEG frames (JPEG per frame)")
    log("[..] Waiting for first packet from ESP32...")
    log("     Press Ctrl+C to quit, Q in image window to quit.")

    packets_received = 0
    chunk_buffer = {}  # { frame_num: { chunk_idx: payload_bytes } }

    try:
        while True:
            # --- receive one datagram ---
            try:
                data, addr = sock.recvfrom(HDR_SIZE + 64 * 1024)
            except (socket.timeout, TimeoutError):
                log("[WAIT] No data in 5s — check: ESP32 running, Ethernet connected, IP correct")
                continue

            if len(data) < HDR_SIZE:
                continue

            packets_received += 1
            if packets_received == 1:
                log(f"[OK]  First packet from {addr[0]}:{addr[1]}  ({len(data)} B)")

            # --- parse header ---
            frame_num, chunk_idx, total_chunks, frame_bytes, chunk_bytes = \
                struct.unpack_from(HDR_FMT, data, 0)
            payload = data[HDR_SIZE : HDR_SIZE + chunk_bytes]

            # --- accumulate chunks ---
            if frame_num not in chunk_buffer:
                chunk_buffer[frame_num] = {}
            chunk_buffer[frame_num][chunk_idx] = payload

            # discard frames that are too old to ever complete
            for old in [k for k in chunk_buffer if k < frame_num - 4]:
                del chunk_buffer[old]

            # --- reassemble + decode when all chunks arrived ---
            if len(chunk_buffer[frame_num]) == total_chunks:
                jpeg = b"".join(chunk_buffer[frame_num][i] for i in range(total_chunks))
                del chunk_buffer[frame_num]

                if len(jpeg) != frame_bytes:
                    log(f"[WARN] Frame {frame_num}: size {len(jpeg)} != {frame_bytes}, skipping")
                    continue

                img = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    log(f"[WARN] Frame {frame_num}: JPEG decode failed, skipping")
                    continue

                cv2.imshow("ESP32-P4 Camera", img)
                log(f"Frame {frame_num:6d}  {len(jpeg):,} B JPEG  from {addr[0]}")

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        log("\n[--] Ctrl+C — exiting.")
    finally:
        sock.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
