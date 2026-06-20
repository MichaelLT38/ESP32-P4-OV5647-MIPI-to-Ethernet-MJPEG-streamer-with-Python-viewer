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

    # save frames outside the project (default: %USERPROFILE%\Pictures\esp32cam):
    python viewer_win.py --save-every 1
    python viewer_win.py --save-dir "D:\captures" --save-every 15
    python viewer_win.py --no-save        # view only, write nothing to disk
"""

import argparse
import os
import socket
import struct
import sys
from datetime import datetime
from pathlib import Path

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
    ap.add_argument("--save-dir", default=str(Path.home() / "Pictures" / "esp32cam"),
                    help="Base folder for saved frames (kept OUTSIDE the project). "
                         "A timestamped subfolder is created per run. "
                         "Default: %%USERPROFILE%%\\Pictures\\esp32cam")
    ap.add_argument("--save-every", type=int, default=0, metavar="N",
                    help="Save every Nth decoded frame as a numbered .jpg "
                         "(0 = don't save numbered frames). latest.jpg is always "
                         "updated while saving is enabled.")
    ap.add_argument("--no-save", action="store_true",
                    help="View only; do not write anything to disk.")
    args = ap.parse_args()

    # --- set up the (out-of-project) save folder ---
    save_dir = None
    if not args.no_save:
        base = Path(args.save_dir).expanduser()
        save_dir = base / datetime.now().strftime("session_%Y%m%d_%H%M%S")
        try:
            save_dir.mkdir(parents=True, exist_ok=True)
            log(f"[OK]  Saving frames to {save_dir}")
            if args.save_every <= 0:
                log("      (numbered frames off; only latest.jpg is updated — use "
                    "--save-every N to keep a history)")
        except OSError as e:
            log(f"[WARN] Could not create save dir {save_dir}: {e} — saving disabled")
            save_dir = None

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

                # --- save the original JPEG bytes (no re-encode) ---
                if save_dir is not None:
                    try:
                        # always keep a rolling latest.jpg (atomic replace)
                        tmp = save_dir / "latest.jpg.tmp"
                        tmp.write_bytes(jpeg)
                        os.replace(tmp, save_dir / "latest.jpg")
                        # optionally keep a numbered history
                        if args.save_every > 0 and (frame_num % args.save_every) == 0:
                            (save_dir / f"frame_{frame_num:06d}.jpg").write_bytes(jpeg)
                    except OSError as e:
                        log(f"[WARN] Frame {frame_num}: save failed ({e})")

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
