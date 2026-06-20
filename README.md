| Supported Targets | ESP32-P4 |
| ----------------- | -------- |

# ESP32-P4 MIPI Camera → Ethernet Video Streamer

Capture live video from a MIPI-CSI camera sensor on the ESP32-P4 and stream the
raw frames over Ethernet (UDP) to a PC for viewing or recording.

> This project started from Espressif's `mipi_isp_dsi` CSI→ISP→DSI display
> example, but the on-board DSI display path has been removed and replaced with a
> chunked UDP video stream over the ESP32-P4 internal Ethernet MAC. The firmware
> entry point is [main/cam_csi_eth_stream_main.c](main/cam_csi_eth_stream_main.c).

## Pipeline

```
MIPI-CSI camera ──▶ ISP (RAW8 passthrough) ──▶ PSRAM frame buffers ──▶ UDP/Ethernet ──▶ PC
   (OV5647)            (no color conv.)          (double-buffered)        (1400 B chunks)
```

## How it works

The firmware runs two cooperating tasks plus the camera DMA ISR:

- **Camera capture (CSI + ISP), callback mode.**
  Two PSRAM DMA buffers (`buf[0]`/`buf[1]`) alternate so one buffer is always
  free for the camera while the other is being read. The DMA done-ISR pushes the
  just-completed buffer into a length-1 overwrite queue. `esp_cam_ctlr_receive()`
  is never called — the ISR drives the next transfer via `on_get_new_trans`.
- **Capture loop (`app_main`).**
  Always services the camera promptly so the pipeline never stalls. When the send
  task is idle it copies the latest frame into a send buffer and signals the send
  task; otherwise it drops the frame.
- **Send task.**
  Splits each frame into 1400-byte UDP datagrams (one Ethernet frame each, to
  avoid IP fragmentation). Every datagram carries a 16-byte header so the receiver
  can reassemble frames:

  ```c
  typedef struct __attribute__((packed)) {
      uint32_t frame_num;
      uint16_t chunk_idx;
      uint16_t total_chunks;
      uint32_t frame_bytes;
      uint32_t chunk_bytes;
  } frame_chunk_hdr_t;   // little-endian, struct format "<IHHII"
  ```

This producer/consumer split decouples the camera rate (50 fps ≈ 20 ms/frame)
from the slower Ethernet send time (~60 ms/frame on 100 Mbps), so frames are
dropped rather than stalling the camera.

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
| UDP destination IP | `UDP_DEST_IP` in [main/cam_csi_eth_stream_main.c](main/cam_csi_eth_stream_main.c) | `192.168.0.94` |
| UDP destination port | `UDP_DEST_PORT` | `12345` |
| UDP chunk size | `UDP_CHUNK_SIZE` | `1400` |
| Camera resolution | `menuconfig` → *Example Configuration* | 800 × 640 |
| Camera format | derived in [main/example_config.h](main/example_config.h) | RAW8 50 fps |
| CSI lane bitrate | `EXAMPLE_MIPI_CSI_LANE_BITRATE_MBPS` | 200 Mbps |
| MIPI PHY LDO | `menuconfig` → *Example Configuration* | chan 3, 2500 mV |

The board obtains its IP via DHCP; the firmware waits for an address before
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
I (xxxx) cam_eth: Frame N: sent <bytes> B (<chunks> chunks)
```

## PC-side receivers

Two helper scripts in [tools/](tools/) reassemble the UDP chunks. Make sure the
`--width`/`--height` match the camera resolution configured on the device and
that any firewall allows UDP on the chosen port.

- **Live viewer (Windows/desktop)** — debayers RAW8 and displays the stream:
  ```
  pip install opencv-python numpy
  python tools/viewer_win.py --port 12345 --width 800 --height 640 --bayer BGGR
  ```
- **Headless recorder (Linux/NAS)** — saves frames to disk (also writes
  `latest.png`):
  ```
  pip3 install numpy Pillow
  python3 tools/recorder_nas.py --port 12345 --width 800 --height 640 --outdir /tmp/frames
  ```

> The on-wire pixel format is RAW8 Bayer (OV5647 is BGGR). Use `viewer_win.py`
> for live RAW8 debayering. `recorder_nas.py` currently assumes an RGB565
> payload, so adjust its decoder if you record the RAW8 stream directly.

## Project layout

```
cam_csi_eth_stream/
├── main/
│   ├── cam_csi_eth_stream_main.c   firmware (capture + UDP stream)
│   ├── example_config.h            pin / camera-format constants
│   ├── Kconfig.projbuild           resolution + LDO options
│   └── CMakeLists.txt              component sources + REQUIRES
├── tools/
│   ├── viewer_win.py               live RAW8 viewer (OpenCV)
│   └── recorder_nas.py             headless frame recorder
└── CMakeLists.txt                  top-level project
```

## References

- [ESP-IDF: Camera Controller Driver](https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/peripherals/camera_driver.html)
- [ESP-IDF: Ethernet](https://docs.espressif.com/projects/esp-idf/en/latest/api-reference/network/esp_eth.html)
- [ESP camera sensor driver](https://components.espressif.com/components/espressif/esp_cam_sensor)
- [ESP-IDF Getting Started](https://docs.espressif.com/projects/esp-idf/en/latest/get-started/index.html)
