/*
 * Goodix GDIX51C0 SPI driver for libfprint
 *
 * Copyright (C) 2026 Berke Kabagoz
 *
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 2.1 of the License, or (at your option) any later version.
 */

#pragma once

#include <config.h>

#ifndef HAVE_UDEV
#error "gdix51c0 requires udev"
#endif

#include <fp-device.h>
#include <fpi-device.h>

G_DECLARE_FINAL_TYPE (FpiDeviceGdix51c0, fpi_device_gdix51c0, FPI, DEVICE_GDIX51C0, FpDevice);

/* Sensor geometry. The MCU pushes a 7690-byte cmd 0x22 payload that wraps
 * a 7680-byte 12bpp frame (5120 px) plus 4-byte CRC + 1-byte tail.
 * Layout: the fast (inner) axis is 64 columns, the slow axis is 80 rows
 * — confirmed by Python image_decode reshape((80, 64)) producing a
 * recognizable fingerprint. */
#define GDIX51C0_SENSOR_WIDTH   64
#define GDIX51C0_SENSOR_HEIGHT  80
#define GDIX51C0_FRAME_PIXELS   (GDIX51C0_SENSOR_WIDTH * GDIX51C0_SENSOR_HEIGHT)
#define GDIX51C0_FRAME_BYTES    7680   /* 5120 px * 1.5 bytes/px */

/* Raw SIGFM enrollment/matching.  We store native 64x80 8-bit frames in
 * FPI_PRINT_RAW templates and match them with SIFT instead of NBIS minutiae. */
#define GDIX51C0_ENROLL_SAMPLES       10
#define GDIX51C0_SIGFM_THRESHOLD      5
#define GDIX51C0_SIGFM_BEST_MIN       10
#define GDIX51C0_SIGFM_MIN_SAMPLES    2

/* GPIO defaults — match protocol_interaction.py. The proper fix is to
 * resolve the IRQ line from the ACPI _CRS, but for our laptop these
 * values are stable. Override with env vars. */
#define GDIX51C0_DEFAULT_GPIOCHIP   "/dev/gpiochip0"
#define GDIX51C0_DEFAULT_IRQ_LINE   321
#define GDIX51C0_DEFAULT_RESET_LINE 140

#define GDIX51C0_ENV_GPIOCHIP   "GDIX51C0_GPIOCHIP"
#define GDIX51C0_ENV_IRQ_LINE   "GDIX51C0_IRQ_LINE"
#define GDIX51C0_ENV_RESET_LINE "GDIX51C0_RESET_LINE"

/* The DPAPI-decrypted, host-side TLS PSK. 32 bytes hex.
 * Per-device (DPAPI uses the machine secret) — hand-deliver via env.
 * Re-deriving the PSK on Linux is a separate problem (see brute_psk_hash.py). */
#define GDIX51C0_ENV_PSK_HEX "GOODIX_TLS_PSK_HEX"

/* Outer SPI wrapper bytes */
#define GDIX51C0_PKT_WRITE 0xa0
#define GDIX51C0_PKT_READ  0xb0

/* Inner command bytes we care about (cmd_byte = (cmd0<<4) | (cmd1<<1)) */
#define GDIX51C0_CMD_NAV_BASE   0x50
#define GDIX51C0_CMD_FDT_DOWN   0x32
#define GDIX51C0_CMD_FDT_UP     0x34
#define GDIX51C0_CMD_FDT_MANUAL 0x36
#define GDIX51C0_CMD_IMAGE_T0   0x20  /* background */
#define GDIX51C0_CMD_IMAGE_T1   0x22  /* with finger */

#define GDIX51C0_BOOT_TIMEOUT_USEC    (3   * 1000 * 1000)
#define GDIX51C0_FDT_TIMEOUT_USEC     (3   * 1000 * 1000)
#define GDIX51C0_IMAGE_TIMEOUT_USEC   (10  * 1000 * 1000)
#define GDIX51C0_FINGER_TIMEOUT_USEC  (60  * 1000 * 1000)

/* spidev xfer chunk size — Linux SPI_IOC_MESSAGE caps at one page. */
#define GDIX51C0_SPI_CHUNK 2048

/* SPI bus settings — match protocol_interaction.py */
#define GDIX51C0_SPI_SPEED_HZ 10000000  /* 10 MHz */
#define GDIX51C0_SPI_MODE     0         /* CPOL=0, CPHA=0 */

#define GDIX51C0_UDEV_TYPES FPI_DEVICE_UDEV_SUBTYPE_SPIDEV

static const FpIdEntry gdix51c0_id_table[] = {
  { .udev_types = GDIX51C0_UDEV_TYPES, .spi_acpi_id = "GDIX51C0" },
  { .udev_types = 0 }
};
