| Supported Targets | ESP32-P4 |
| ----------------- | -------- |

# ESP32-P4 MIPI Camera → Ethernet Video Streamer (MJPEG)

Capture live 1080p30 video from a MIPI-CSI camera sensor on the ESP32-P4,
compress each frame to JPEG with the on-chip hardware encoder, and stream the
compressed frames over Ethernet (UDP) to a PC for viewing or recording.

> This project started from Espressif's `mipi_isp_dsi` CSI→ISP→DSI display
> example, but the on-board DSI display path has been removed and replaced with a
> chunked UDP MJPEG stream over the ESP32-P4 internal Ethernet MAC. The firmware
> entry point is [main/cam_csi_eth_stream_main.c](main/cam_csi_eth_stream_main.c).

## Pipeline

```
MIPI-CSI ──▶ ISP ──────────▶ PSRAM ──▶ JPEG encoder ─────▶ UDP/Ethernet ──▶ PC
 (OV5647)    (RAW10→RGB565)  buffers   (RGB565 in,          (1400 B chunks)   (decode
  RAW10                                 4:2:0 JPEG out)                        JPEG)
  1920×1080 @ 30 fps
```

The "compression" is the hardware JPEG encoder (`esp_driver_jpeg`). The camera
emits RAW10 Bayer, the ISP demosaics it to packed RGB565, and the JPEG engine
compresses that to a 4:2:0 YUV JPEG. Output is true MJPEG: one self-contained
JPEG per video frame.

> **Why RGB565 into the encoder (not YUV422 directly)?** The ESP32-P4 ISP emits
> YUV422 in **UYVY** byte order, while the JPEG encoder's YUV422 input expects
> **YVYU** — a byte-order mismatch whose reorder path is chip-revision
> dependent. ISP→RGB565→JPEG is the configuration Espressif's official OV5647
> test verifies, and the JPEG *output* is still 4:2:0 YUV, so compression ratio
> and quality are unchanged. To switch to a YUV422 encoder input later, change
> the ISP output to `ISP_COLOR_YUV422` and the JPEG `src_type` to
> `JPEG_ENCODE_IN_FORMAT_YUV422` (and verify colors on hardware).

## How it works

The firmware runs two cooperating tasks plus the camera DMA ISR:

- **Camera capture (CSI + ISP), callback mode.**
  Two PSRAM DMA buffers (`buf[0]`/`buf[1]`) alternate so one buffer is always
  free for the camera while the other is being read. The DMA done-ISR pushes the
  just-completed buffer into a length-1 overwrite queue. `esp_cam_ctlr_receive()`
  is never called — the ISR drives the next transfer via `on_get_new_trans`.
- **Capture loop (`app_main`).**
  Always services the camera promptly so the pipeline never stalls. When the
  encode/send task is idle it copies the latest frame into the JPEG input buffer
  and signals the task; otherwise it drops the frame.
- **Encode + send task.**
  JPEG-encodes the snapshot (RGB565 → 4:2:0 JPEG), then splits the compressed
  bitstream into 1400-byte UDP datagrams (one Ethernet frame each, to avoid IP
  fragmentation). Every datagram carries a 16-byte header so the receiver can
  reassemble frames; `frame_bytes` is the size of the whole compressed JPEG:

  ```c
  typedef struct __attribute__((packed)) {
      uint32_t frame_num;
      uint16_t chunk_idx;
      uint16_t total_chunks;
      uint32_t frame_bytes;
      uint32_t chunk_bytes;
  } frame_chunk_hdr_t;   // little-endian, struct format "<IHHII"
  ```

This producer/consumer split decouples the camera rate (30 fps ≈ 33 ms/frame)
from the JPEG-encode + Ethernet send time, so frames are dropped rather than
stalling the camera. A compressed 1080p JPEG at quality 80 is roughly
150–400 KB, so a 30 fps stream is on the order of ~5–12 MB/s — comfortably
within a 100 Mbps link, versus ~93 MB/s for uncompressed RGB565 1080p30.

> **Note:** The MIPI D-PHY is powered by an internal LDO channel. The firmware
> acquires LDO channel 3 @ 2500 mV before using the CSI controller. Without this,
> the sensor still answers on I²C (SCCB) but no CSI data ever arrives.

## Hardware required

- ESP32-P4 devkit (with PSRAM)
- OV5647 MIPI-CSI camera module (2-lane)
- Ethernet PHY on the board (internal EMAC). Verify your PHY in
  [main/cam_csi_eth_stream_main.c](main/cam_csi_eth_stream_main.c) — the default
  is `esp_eth_phy_new_generic()`; swap for `esp_eth_phy_new_rtl8201()` or
  `esp_eth_phy_new_lan87xx()` as needed.
- A PC on the same network to receive the stream.

### Default I/O

| Signal | Pin |
| ------ | --- |
| SCCB SCL | GPIO 8 |
| SCCB SDA | GPIO 7 |

(See [main/example_config.h](main/example_config.h).)

## Configuration

Key compile-time settings:

| Setting | Where | Default |
| ------- | ----- | ------- |
| UDP destination IP | `menuconfig` → *Example Configuration* (`EXAMPLE_UDP_DEST_IP`) | `192.168.1.100` (set to your PC) |
| UDP destination port | `menuconfig` → *Example Configuration* (`EXAMPLE_UDP_DEST_PORT`) | `12345` |
| UDP chunk size | `UDP_CHUNK_SIZE` in [main/cam_csi_eth_stream_main.c](main/cam_csi_eth_stream_main.c) | `1400` |
| Camera resolution | `menuconfig` → *Example Configuration* | 1280 × 960 (binning) |
| Camera format | derived in [main/example_config.h](main/example_config.h) | RAW10 45 fps |
| CSI lane bitrate | `EXAMPLE_MIPI_CSI_LANE_BITRATE_MBPS` | 221 Mbps (1280×960) |
| JPEG quality | `menuconfig` → *Example Configuration* (`EXAMPLE_JPEG_QUALITY`) | 85 (1–100) |
| MIPI PHY LDO | `menuconfig` → *Example Configuration* | chan 3, 2500 mV |

Set `EXAMPLE_UDP_DEST_IP` to the IP of the PC running the viewer/recorder. The
board obtains its own IP via DHCP; the firmware waits for an address before
starting the stream.

## Build, flash, monitor

```
idf.py set-target esp32p4
idf.py menuconfig      # set resolution; confirm LDO channel/voltage
idf.py build flash monitor
```

Exit the monitor with `Ctrl` + `]`.

Expected serial output once running:

```
I (xxxx) cam_eth: Ethernet got IP: 192.168.0.xx
I (xxxx) cam_eth: Network ready
I (xxxx) cam_eth: MIPI PHY LDO acquired (chan 3, 2500 mV)
I (xxxx) cam_eth: Camera started — streaming
I (xxxx) cam_eth: Frame N queued (sent so far: ...)
I (xxxx) cam_eth: Frame N: JPEG <bytes> B (<chunks> chunks)
```

## PC-side receivers

Two helper scripts in [tools/](tools/) reassemble the UDP chunks and handle the
JPEG payload. No resolution or Bayer settings are needed — each JPEG is
self-describing. Make sure any firewall allows UDP on the chosen port.

- **Live viewer (Windows/desktop)** — decodes and displays the MJPEG stream:
  ```
  pip install opencv-python numpy
  python tools/viewer_win.py --port 12345
  ```
- **Headless recorder (Linux/NAS)** — saves each frame straight to a `.jpg`
  (also writes `latest.jpg`); standard library only, no decode needed:
  ```
  python3 tools/recorder_nas.py --port 12345 --outdir /tmp/frames
  ```

## Project layout

```
cam_csi_eth_stream/
├── main/
│   ├── cam_csi_eth_stream_main.c   firmware (capture + UDP stream)
│   ├── example_config.h            pin / camera-format constants
│   ├── Kconfig.projbuild           resolution + LDO options
│   └── CMakeLists.txt              component sources + REQUIRES
├── tools/
│   ├── viewer_win.py               live MJPEG viewer (OpenCV decode)
│   └── recorder_nas.py             headless JPEG recorder (no decode)
└── CMakeLists.txt                  top-level project
```

## References

- [ESP-IDF: Camera Controller Driver](https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/camera_driver.html)
- [ESP-IDF: JPEG Encoder/Decoder](https://docs.espressif.com/projects/esp-idf/en/latest/esp32p4/api-reference/peripherals/jpeg.html)
- [ESP-IDF: Ethernet](https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/network/esp_eth.html)
- [ESP camera sensor driver](https://components.espressif.com/components/espressif/esp_cam_sensor)
- [ESP-IDF Getting Started](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html)
