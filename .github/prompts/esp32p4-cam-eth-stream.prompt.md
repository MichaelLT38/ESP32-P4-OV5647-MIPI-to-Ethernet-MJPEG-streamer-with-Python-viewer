---
mode: agent
description: >
  Context prompt for the ESP32-P4 (Waveshare) MIPI camera → Ethernet UDP streaming project.
  Use when resuming work on the camera pipeline, UDP streaming, Python viewer, or debugging
  the CSI/ISP driver on this board.
---

# ESP32-P4 MIPI Camera → Ethernet UDP Stream — Project Context

## Hardware

| Component | Detail |
|-----------|--------|
| Board | Waveshare ESP32-P4 |
| SoC | ESP32-P4, RISC-V, 360 MHz, 32 MB PSRAM |
| Camera | OV5647, MIPI CSI-2, 2-lane, I²C on SDA=GPIO7/SCL=GPIO8 |
| Active mode | `MIPI_2lane_24Minput_RAW10_1280x960_binning_45fps` |
| Bayer pattern | **GBRG** (set via ISP `bayer_order`) |
| Network | 100 Mbps Ethernet, generic PHY (`esp_eth_phy_new_generic`), DHCP |
| ESP32 IP | DHCP (varies) |
| PC IP | set via `EXAMPLE_UDP_DEST_IP` in menuconfig, UDP port 12345 |

## Software environment

- **ESP-IDF** v6.0.1 (`C:\esp\v6.0.1\esp-idf`)
- **Toolchain** riscv32-esp-elf GCC 15.2 (`C:\Espressif`)
- **Target** `esp32p4`
- Build tool: `idf.py build flash monitor`

## Project layout

```
cam_csi_eth_stream/
  main/
    cam_csi_eth_stream_main.c   ← firmware (main source)
    CMakeLists.txt         ← REQUIRES: esp_mm esp_driver_isp esp_driver_cam
                                       esp_driver_i2c sensor_init esp_netif
                                       esp_eth lwip freertos
    example_config.h       ← pin/format constants
  tools/
    viewer_win.py          ← Windows OpenCV viewer (UDP reassembly + debayer)
  managed_components/      ← espressif__esp_cam_sensor (OV5647), etc.
```

## Status: WORKING (continuous live stream confirmed 2026-06-15)

## Architecture

```
[esp_ldo_acquire_channel(chan 3, 2500mV) powers MIPI PHY  <-- MANDATORY]
OV5647 → MIPI CSI-2 → CSI ctrl (RAW8 passthrough) → ISP (RAW8 passthrough)
       → DMA → PSRAM buf[0/1] → (on_trans_finished ISR) → capture task
               → memcpy → s_send_buf → send_task → UDP chunks → PC
```

- **Double PSRAM DMA buffers** (`buf[0]`, `buf[1]`) — driver alternates via `trans_que`
- **`s_send_buf`** — 3rd PSRAM buffer, snapshot of last completed frame
- **`s_frame_q`** — FreeRTOS queue (length 2), ISR (`on_trans_finished`) pushes completed buffer ptr
- **`s_rdy`/`s_done`** — binary semaphores for capture ↔ send_task handshake
- **`send_task`** (priority 3) — waits on `s_rdy`, sends 1400-byte UDP chunks from `s_send_buf`

## Critical driver facts (read the source before changing)

File: `C:\esp\v6.0.1\esp-idf\components\esp_driver_cam\csi\src\esp_cam_ctlr_csi.c`

### ISR (`csi_dma_trans_done_callback`) logic

1. If `on_get_new_trans` registered → calls it for the next buffer (**callback mode**)
2. Else → pops from `ctlr->trans_que` via `xQueueReceiveFromISR` (**queue mode**)
3. If neither yields a buffer → falls back to `ctlr->backup_buffer`
4. Calls `on_trans_finished` (if registered) with the **previous** completed buffer

### `esp_cam_ctlr_receive()` — queue mode only

- Pushes a `trans` struct into `ctlr->trans_que` (internal driver queue, `queue_items=2`)
- **Only works when `on_get_new_trans` is NULL** — if callback mode is active the ISR
  never reads `trans_que`, so the queue fills after 2 calls → error 263 (`ESP_ERR_TIMEOUT`)

### `esp_cam_ctlr_start()` — seeds the first DMA transfer

- If `on_get_new_trans` registered: calls it to get the first buffer
- Else: pops one entry from `trans_que` **or** uses `backup_buffer`
- Therefore in queue mode, call `receive()` to pre-load buffers **before** `start()`

### Correct queue-mode startup sequence

```c
// 1. Create controller (queue_items=2)
// 2. Register callbacks: on_get_new_trans=NULL, on_trans_finished=<your ISR>
// 3. esp_cam_ctlr_enable()
// 4. esp_cam_ctlr_receive(buf[0])  ← pre-load slot 0
// 5. esp_cam_ctlr_receive(buf[1])  ← pre-load slot 1 (optional but fills pipeline)
// 6. esp_cam_ctlr_start()          ← pops buf[0] → first DMA → ISR fires → pops buf[1]
// 7. Loop: xQueueReceive(s_frame_q) → copy → recycle via receive() → repeat
```

### Why `on_trans_finished` fires with the *previous* buffer

The ISR first configures & starts the **next** DMA transfer (getting a new buffer), then
calls `on_trans_finished` on `ctlr->trans` (the buffer that just finished). This means
by the time your callback runs, DMA into the next buffer has already started — the
finished buffer is safe to read/copy.

## UDP framing

```c
typedef struct __attribute__((packed)) {
    uint32_t frame_num;     // monotonic counter
    uint16_t chunk_idx;     // 0-based
    uint16_t total_chunks;
    uint32_t frame_bytes;   // total frame size
    uint32_t chunk_bytes;   // payload in this datagram
} frame_chunk_hdr_t;        // 16 bytes
```

- Chunk payload: **1400 bytes** (fits in one Ethernet frame, avoids IP fragmentation
  which Windows Defender blocks for large fragments)
- lwIP cannot DMA from PSRAM → `pkt` staging buffer allocated with
  `MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT`

## Python viewer (`tools/viewer_win.py`)

```bash
python viewer_win.py --width 800 --height 640 --bayer BGGR
```

- Bayer choices: `BGGR` (OV5647 default), `RGGB`, `GBRG`, `GRBG`
- Reassembles chunks by `frame_num`, debayers with OpenCV `cv2.COLOR_BayerBGGR2BGR`
- Frame size = `width × height × 1` (RAW8 = 1 byte/pixel)

## Known issues & fixes applied

| Symptom | Root cause | Fix |
|---------|------------|-----|
| Error 263 / `csi recv API queue is full` | Called `receive()` while `on_get_new_trans` was registered (callback mode ignores trans_que) | Remove `on_get_new_trans` callback |
| "No frame in 1 s — camera stalled?" | Queue mode but no buffers pre-loaded before `start()` — hardware had no DMA descriptor | Pre-load both buffers with `receive()` before `start()` |
| Pipeline stall after 2 frames | Completed buffers never recycled — driver ran out of destinations | Call `receive(frame_buf)` at bottom of capture loop |
| `ENETDOWN` on first `sendto` | Socket created before DHCP completed | EventGroup wait on `IP_EVENT_ETH_GOT_IP` |
| Windows not receiving large packets | IP fragmentation of 32 KB chunks blocked by Windows Defender | Reduced chunk size to 1400 bytes |
| Wrong colors / noise | Wrong Bayer pattern (GBRG used, OV5647 is BGGR) | `--bayer BGGR` in viewer |
| Build: duplicate definitions | Previous rewrite appended instead of replacing | Truncated file to first 356 lines |
| Build: `esp_eth_mac_new_esp32` too few args | Missing `eth_mac_config_t` second arg | Add `ETH_MAC_DEFAULT_CONFIG()` |
| ISP init failed [262] | ISP enabled before CSI controller created | Create ISP → create CSI → enable ISP |

## ISP / CSI init order (critical)

```c
esp_isp_new_processor(...)   // CREATE isp (do NOT enable yet)
esp_cam_new_csi_ctlr(...)    // CREATE csi (links to isp in init state)
esp_isp_enable(isp_proc)     // ENABLE isp (now safe)
// register callbacks, enable ctlr, pre-load receive, start
```

## sdkconfig resolution

- `CONFIG_EXAMPLE_MIPI_CSI_DISP_HRES=800`
- `CONFIG_EXAMPLE_MIPI_CSI_DISP_VRES=640`
- OV5647 has no 1024×600 mode; 800×640 is the closest 50fps RAW8 mode

## THE bug that cost the most time: missing MIPI PHY LDO

The original DSI example acquired an internal LDO regulator to power the MIPI D-PHY.
When DSI code was stripped, the LDO went too. Without it:
  - sensor answers on I2C/SCCB (PID=0x5647 detected) ✓
  - but CSI data lanes have NO power → no bytes → DMA-done ISR never fires
  - → "No frame in 1 s" forever
Fix: `esp_ldo_acquire_channel({.chan_id=3, .voltage_mv=2500}, &h)` before camera init.
Header `esp_ldo_regulator.h` is in component **esp_hw_support** (NOT esp_driver_ldo —
that name fails CMake with "unknown name"). Add `esp_hw_support` to CMakeLists REQUIRES.

The earlier "garbled image" was uninitialized PSRAM, not real camera data —
`esp_cam_ctlr_receive()` does NOT block for a frame, it just queues a buffer and returns.

## Ideas to work on next (TODO)

- **Confirm Bayer pattern** — verify BGGR gives natural colours; try RGGB if off.
- **Frame pacing / drop counter** — log fps actually delivered to PC vs captured.
- **JPEG / RLE compression** before UDP to cut 512 KB → fit more fps over 100 Mbps.
- **Sequence/loss handling in viewer** — detect dropped chunks, skip incomplete frames.
- **Identify the real PHY** on the Waveshare board, replace `esp_eth_phy_new_generic`.
- **Use ISP debayer → RGB565 on-chip** if a future need; currently RAW8 passthrough.
- **Make UDP_DEST_IP configurable** (Kconfig / runtime) instead of hard-coded.

## Current status (working)

- ✅ MIPI PHY LDO acquired — camera streams continuously
- ✅ Build clean, flashes successfully
- ✅ Ethernet DHCP, ARP warmup
- ✅ Camera sensor detected (OV5647 PID=0x5647), correct format selected
- ✅ Continuous frames over UDP, Python viewer shows live image
- ✅ Python viewer functional, BGGR Bayer pattern
