"""
ESP32-P4 camera UDP frame viewer (Windows)
Receives chunked RAW8 (Bayer GBRG) frames and displays them live.

Usage:
    pip install opencv-python numpy
    python viewer_win.py [--port 12345] [--width 800] [--height 640]
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
HDR_FMT  = "<IHHII"
HDR_SIZE = struct.calcsize(HDR_FMT)   # 16 bytes

def raw8_to_bgr(data: bytes, width: int, height: int, bayer_code: int) -> np.ndarray:
    """Debayer RAW8 Bayer data to BGR using OpenCV."""
    bayer = np.frombuffer(data, dtype=np.uint8).reshape(height, width)
    return cv2.cvtColor(bayer, bayer_code)

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port",   type=int, default=12345)
    ap.add_argument("--width",  type=int, default=800)
    ap.add_argument("--height", type=int, default=640)
    ap.add_argument("--bayer",  default="BGGR",
                    choices=["BGGR","RGGB","GBRG","GRBG"],
                    help="Bayer pattern (OV5647 is BGGR)")
    args = ap.parse_args()

    bayer_codes = {
        "BGGR": cv2.COLOR_BayerBGGR2BGR,
        "RGGB": cv2.COLOR_BayerRGGB2BGR,
        "GBRG": cv2.COLOR_BayerGBRG2BGR,
        "GRBG": cv2.COLOR_BayerGRBG2BGR,
    }
    bayer_code = bayer_codes[args.bayer]

    try:
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(("0.0.0.0", args.port))
        sock.settimeout(5.0)
    except OSError as e:
        log(f"[ERR]  Could not bind UDP port {args.port}: {e}")
        log("       Is another instance of this script already running?")
        sys.exit(1)

    log(f"[OK]  Socket bound on 0.0.0.0:{args.port}")
    log(f"[OK]  Expecting {args.width}x{args.height} RAW8/Bayer frames")
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

            # --- reassemble when all chunks arrived ---
            if len(chunk_buffer[frame_num]) == total_chunks:
                raw = b"".join(chunk_buffer[frame_num][i] for i in range(total_chunks))
                del chunk_buffer[frame_num]

                expected = args.width * args.height * 1  # RAW8 = 1 byte/pixel
                if len(raw) != expected:
                    log(f"[WARN] Frame {frame_num}: size {len(raw)} != {expected}, skipping")
                    continue

                img = raw8_to_bgr(raw, args.width, args.height, bayer_code)
                cv2.imshow("ESP32-P4 Camera", img)
                log(f"Frame {frame_num:6d}  {len(raw):,} B  from {addr[0]}")

                if cv2.waitKey(1) & 0xFF == ord("q"):
                    break

    except KeyboardInterrupt:
        log("\n[--] Ctrl+C — exiting.")
    finally:
        sock.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
