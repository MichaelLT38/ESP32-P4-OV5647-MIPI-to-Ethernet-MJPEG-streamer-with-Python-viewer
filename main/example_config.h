/*
 * SPDX-FileCopyrightText: 2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include "sdkconfig.h"

#ifdef __cplusplus
extern "C" {
#endif

#define EXAMPLE_RGB565_BITS_PER_PIXEL           16
#define EXAMPLE_RGB565_BYTES_PER_PIXEL          (EXAMPLE_RGB565_BITS_PER_PIXEL / 8)

#define EXAMPLE_MIPI_CSI_CAM_SCCB_SCL_IO        (8)
#define EXAMPLE_MIPI_CSI_CAM_SCCB_SDA_IO        (7)

#if CONFIG_EXAMPLE_MIPI_CSI_HRES_1280

/* OV5647 1280x960 uses on-sensor 2x2 binning: pixels are averaged in pairs,
 * which roughly halves read noise and improves low-light legibility (ideal for
 * security) while running faster (45 fps, 1.23 MP vs 2.07 MP). Still RAW10, so
 * the CSI-passthrough + ISP-demosaic pipeline is unchanged. Lane bit rate from
 * the driver's OV5647_MIPI_CSI_LINE_RATE_1280x960_45FPS (88.3333 MHz IDI * 5),
 * in the CSI half-rate convention. */
#define EXAMPLE_CAM_FORMAT                     "MIPI_2lane_24Minput_RAW10_1280x960_binning_45fps"
#define EXAMPLE_MIPI_CSI_LANE_BITRATE_MBPS     221

#elif CONFIG_EXAMPLE_MIPI_CSI_HRES_1920

/* OV5647 1080p is a RAW10 mode (not RAW8). Lane bit rate follows the sensor
 * driver's OV5647_MIPI_CSI_LINE_RATE_1920x1080_30FPS (81.6667 MHz IDI * 5),
 * expressed in the CSI controller's half-rate convention (matches the way the
 * official 800x640 test maps its 400 MHz line rate to a 200 Mbps config). */
#define EXAMPLE_CAM_FORMAT                     "MIPI_2lane_24Minput_RAW10_1920x1080_30fps"
#define EXAMPLE_MIPI_CSI_LANE_BITRATE_MBPS     204

#elif CONFIG_EXAMPLE_MIPI_CSI_HRES_800

#define EXAMPLE_MIPI_CSI_LANE_BITRATE_MBPS      200 //matches Espressif OV5647 800x640 test

#if CONFIG_EXAMPLE_MIPI_CSI_VRES_640
#define EXAMPLE_CAM_FORMAT                     "MIPI_2lane_24Minput_RAW8_800x640_50fps"
#elif CONFIG_EXAMPLE_MIPI_CSI_VRES_800
#define EXAMPLE_CAM_FORMAT                     "MIPI_2lane_24Minput_RAW8_800x800_50fps"
#elif CONFIG_EXAMPLE_MIPI_CSI_VRES_1280
#define EXAMPLE_CAM_FORMAT                     "MIPI_2lane_24Minput_RAW8_800x1280_50fps"
#endif

#elif CONFIG_EXAMPLE_MIPI_CSI_HRES_1024

#define EXAMPLE_MIPI_CSI_LANE_BITRATE_MBPS      200

#if CONFIG_EXAMPLE_MIPI_CSI_VRES_600
#define EXAMPLE_CAM_FORMAT                     "MIPI_2lane_24Minput_RAW8_1024x600_30fps"
#endif

#endif

#ifdef __cplusplus
}
#endif
