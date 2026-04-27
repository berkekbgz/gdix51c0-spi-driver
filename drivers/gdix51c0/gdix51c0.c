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

#define FP_COMPONENT "gdix51c0"

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <gpiod.h>
#include <linux/spi/spidev.h>

#include "drivers_api.h"
#include "fpi-spi-transfer.h"

#include "gdix51c0.h"
#include "gdix51c0-proto.h"
#include "gdix51c0-tls.h"
#include "sigfm/sigfm.hpp"

struct _FpiDeviceGdix51c0
{
  FpDevice                   parent;

  /* hardware handles */
  int                        spi_fd;
  struct gpiod_chip         *gpio_chip;
  struct gpiod_line_request *irq_req;
  unsigned int               irq_offset;
  unsigned int               reset_offset;
  struct gpiod_edge_event_buffer *irq_events;

  /* TLS state (populated for each enroll/verify/identify session) */
  Gdix51c0Tls                   tls;
  gboolean                   tls_ready;

  gboolean                   skip_next_identify;
  gboolean                   session_desynced;

  guint8                     fdt_down_regs[12];
  guint16                    fdt_down_sample[6];
  gboolean                   fdt_down_sample_valid;
};

G_DEFINE_TYPE (FpiDeviceGdix51c0, fpi_device_gdix51c0, FP_TYPE_DEVICE);

#define GDIX51C0_TOUCH_IGNORE 0xff
#define GDIX51C0_FDT_HOST_TOUCH_MIN_DROP 25
#define GDIX51C0_FDT_HOST_TOUCH_TOTAL_DROP 70

static const guint8 gdix51c0_default_fdt_down_regs[12] = {
  0x80, 0xad, 0x80, 0xbb, 0x80, 0xa2,
  0x80, 0xae, 0x80, 0xa3, 0x80, 0xae
};

/* ------------------------------------------------------------------ */
/* GPIO bring-up                                                       */
/* ------------------------------------------------------------------ */

static int
gdix51c0_env_int (const char *name, int fallback)
{
  const char *s = g_getenv (name);
  if (!s || !*s)
    return fallback;
  return (int) g_ascii_strtoll (s, NULL, 0);
}

/* libgpiod v2: chip → line settings → line config → request.  For the
 * IRQ line we also enable both-edge detection so the kernel buffers
 * events for us; that beats polling for short MCU pulses. */
static struct gpiod_line_request *
gdix51c0_request_line (struct gpiod_chip   *chip,
                    unsigned int         offset,
                    enum gpiod_line_direction dir,
                    int                  initial_high,
                    gboolean             want_edges,
                    const char          *consumer,
                    GError             **err)
{
  struct gpiod_line_settings *settings = gpiod_line_settings_new ();
  struct gpiod_line_config   *line_cfg = gpiod_line_config_new ();
  struct gpiod_request_config *req_cfg = gpiod_request_config_new ();
  struct gpiod_line_request  *req      = NULL;

  if (!settings || !line_cfg || !req_cfg)
    {
      g_set_error (err, G_IO_ERROR, G_IO_ERROR_FAILED,
                   "gdix51c0: gpiod alloc failed");
      goto out;
    }

  gpiod_line_settings_set_direction (settings, dir);
  if (dir == GPIOD_LINE_DIRECTION_OUTPUT)
    gpiod_line_settings_set_output_value (settings,
                                          initial_high ? GPIOD_LINE_VALUE_ACTIVE
                                                       : GPIOD_LINE_VALUE_INACTIVE);
  if (want_edges)
    gpiod_line_settings_set_edge_detection (settings, GPIOD_LINE_EDGE_BOTH);

  if (gpiod_line_config_add_line_settings (line_cfg, &offset, 1, settings) < 0)
    {
      g_set_error (err, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: gpiod add line %u failed", offset);
      goto out;
    }
  gpiod_request_config_set_consumer (req_cfg, consumer);

  req = gpiod_chip_request_lines (chip, req_cfg, line_cfg);
  if (!req)
    g_set_error (err, G_IO_ERROR, g_io_error_from_errno (errno),
                 "gdix51c0: request line %u failed", offset);

out:
  if (settings) gpiod_line_settings_free (settings);
  if (line_cfg) gpiod_line_config_free (line_cfg);
  if (req_cfg)  gpiod_request_config_free (req_cfg);
  return req;
}

static gboolean
gdix51c0_open_gpios (FpiDeviceGdix51c0 *self, GError **err)
{
  const char *chip_path = g_getenv (GDIX51C0_ENV_GPIOCHIP);

  if (!chip_path || !*chip_path)
    chip_path = GDIX51C0_DEFAULT_GPIOCHIP;

  self->irq_offset   = gdix51c0_env_int (GDIX51C0_ENV_IRQ_LINE,   GDIX51C0_DEFAULT_IRQ_LINE);
  self->reset_offset = gdix51c0_env_int (GDIX51C0_ENV_RESET_LINE, GDIX51C0_DEFAULT_RESET_LINE);

  self->gpio_chip = gpiod_chip_open (chip_path);
  if (!self->gpio_chip)
    {
      g_set_error (err, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: open gpiochip %s failed", chip_path);
      return FALSE;
    }

  self->irq_req = gdix51c0_request_line (self->gpio_chip, self->irq_offset,
                                      GPIOD_LINE_DIRECTION_INPUT, 0,
                                      TRUE /* edges */,
                                      "gdix51c0-irq", err);
  if (!self->irq_req)
    return FALSE;

  /* Buffer up to 16 pending edges; we typically expect 2-4 per cmd. */
  self->irq_events = gpiod_edge_event_buffer_new (16);
  if (!self->irq_events)
    {
      g_set_error_literal (err, G_IO_ERROR, G_IO_ERROR_FAILED,
                           "gdix51c0: alloc edge event buffer failed");
      return FALSE;
    }

  /* Reset line is opened on-demand inside reset_pulse — see comment
   * there for why we don't keep a persistent OUTPUT request. */
  return TRUE;
}

static void
gdix51c0_close_gpios (FpiDeviceGdix51c0 *self)
{
  g_clear_pointer (&self->irq_req,    gpiod_line_request_release);
  g_clear_pointer (&self->irq_events, gpiod_edge_event_buffer_free);
  g_clear_pointer (&self->gpio_chip,  gpiod_chip_close);
}

/* ------------------------------------------------------------------ */
/* Device open/close                                                   */
/* ------------------------------------------------------------------ */

static void
gdix51c0_open (FpDevice *dev)
{
  FpiDeviceGdix51c0 *self = FPI_DEVICE_GDIX51C0 (dev);
  GError *err = NULL;

  G_DEBUG_HERE ();

  const char *spi_path = fpi_device_get_udev_data (dev, FPI_DEVICE_UDEV_SUBTYPE_SPIDEV);
  fp_dbg ("gdix51c0: opening spidev at %s", spi_path ? spi_path : "(null)");
  if (!spi_path)
    {
      g_set_error_literal (&err, G_IO_ERROR, G_IO_ERROR_NOT_FOUND,
                           "gdix51c0: spidev node not found");
      fpi_device_open_complete (dev, err);
      return;
    }

  self->spi_fd = open (spi_path, O_RDWR);
  if (self->spi_fd < 0)
    {
      g_set_error (&err, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: open spidev %s failed", spi_path);
      fpi_device_open_complete (dev, err);
      return;
    }

  /* Match Python: 10 MHz, mode 0, 8 bits/word.  Without this we inherit
   * whatever the kernel set the fd to. */
  guint32 speed = GDIX51C0_SPI_SPEED_HZ;
  guint8  mode  = GDIX51C0_SPI_MODE;
  guint8  bpw   = 8;
  if (ioctl (self->spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &speed) < 0 ||
      ioctl (self->spi_fd, SPI_IOC_WR_MODE,         &mode)  < 0 ||
      ioctl (self->spi_fd, SPI_IOC_WR_BITS_PER_WORD, &bpw)  < 0)
    {
      g_set_error (&err, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: spidev configure failed");
      close (self->spi_fd);
      self->spi_fd = -1;
      fpi_device_open_complete (dev, err);
      return;
    }
  /* Read back to confirm the kernel honored what we asked for. */
  guint32 actual_speed = 0;
  guint8  actual_mode  = 0xff;
  guint8  actual_bpw   = 0;
  ioctl (self->spi_fd, SPI_IOC_RD_MAX_SPEED_HZ, &actual_speed);
  ioctl (self->spi_fd, SPI_IOC_RD_MODE,         &actual_mode);
  ioctl (self->spi_fd, SPI_IOC_RD_BITS_PER_WORD, &actual_bpw);
  fp_dbg ("gdix51c0: spidev configured: speed=%u Hz mode=0x%02x bpw=%u",
          actual_speed, actual_mode, actual_bpw);

  if (!gdix51c0_open_gpios (self, &err))
    {
      close (self->spi_fd);
      self->spi_fd = -1;
      gdix51c0_close_gpios (self);
      fpi_device_open_complete (dev, err);
      return;
    }

  fpi_device_open_complete (dev, NULL);
}

static void
gdix51c0_close (FpDevice *dev)
{
  FpiDeviceGdix51c0 *self = FPI_DEVICE_GDIX51C0 (dev);

  G_DEBUG_HERE ();

  if (self->tls_ready)
    {
      gdix51c0_tls_free (&self->tls);
      self->tls_ready = FALSE;
    }

  if (self->spi_fd >= 0)
    {
      close (self->spi_fd);
      self->spi_fd = -1;
    }
  gdix51c0_close_gpios (self);

  fpi_device_close_complete (dev, NULL);
}

/* Reset + post-reset IRQ pulse + force-unlock-TLS (cmd 0xd5, no ack). */
static gboolean
gdix51c0_boot_probe (FpiDeviceGdix51c0 *self, GError **error)
{
  Gdix51c0Bus bus = {
    .dev = FP_DEVICE (self), .spi_fd = self->spi_fd,
    .irq_req = self->irq_req, .irq_offset = self->irq_offset,
    .irq_events = self->irq_events,
  };

  gdix51c0_irq_drain (self->irq_req, self->irq_events);
  if (!gdix51c0_reset_pulse (self->gpio_chip, self->reset_offset, error))
    return FALSE;

  if (!gdix51c0_irq_wait (self->irq_req, self->irq_events, TRUE,
                       self->irq_offset, GDIX51C0_BOOT_TIMEOUT_USEC,
                       "boot-rise", error))
    return FALSE;
  if (!gdix51c0_irq_wait (self->irq_req, self->irq_events, FALSE,
                       self->irq_offset, GDIX51C0_BOOT_TIMEOUT_USEC,
                       "boot-fall", error))
    return FALSE;

  /* Settle delay: the rise/fall we just caught may be a GPIO glitch from
   * our reset toggle, not the MCU's actual post-reset boot pulse.  Give
   * the MCU at least 100ms to fully boot, then drain any later edges so
   * the cmd flow starts from a clean kernel buffer. */
  g_usleep (200000);
  gdix51c0_irq_drain (self->irq_req, self->irq_events);

  static const guint8 d5_payload[] = { 0xd5, 0x03, 0x00, 0x00, 0x00, 0xd3 };
  return gdix51c0_cmd_no_ack (&bus, d5_payload, sizeof (d5_payload),
                           "force-unlock-tls", error);
}

static void
gdix51c0_debug_mcu_state_resp (const guint8 *buf, gsize len)
{
  GString *s;

  if (!buf || len == 0)
    {
      fp_dbg ("gdix51c0: mcu-state: empty response");
      return;
    }

  s = g_string_new (NULL);
  for (gsize i = 0; i < len; i++)
    g_string_append_printf (s, "%02x", buf[i]);

  fp_dbg ("gdix51c0: mcu-state raw response len=%zu: %s", len, s->str);
  g_string_free (s, TRUE);

  if (len < 4)
    {
      fp_warn ("gdix51c0: mcu-state response too short: %zu", len);
      return;
    }

  guint8 cmd = buf[0];
  guint16 inner_len = (guint16) buf[1] | ((guint16) buf[2] << 8);
  gsize expected_total = 3 + inner_len;

  if (cmd != 0xae)
    fp_warn ("gdix51c0: mcu-state response cmd is 0x%02x, expected 0xae", cmd);

  if (len != expected_total)
    {
      fp_warn ("gdix51c0: mcu-state total len=%zu, inner_len=%u, expected total=%zu",
               len, inner_len, expected_total);
      return;
    }

  fp_dbg ("gdix51c0: mcu-state packet OK: cmd=0x%02x inner_len=%u total=%zu",
          cmd, inner_len, len);
}

/* Full 14213 pre-TLS init sequence — every byte mirrors
 * protocol_interaction.py from get_evk_version() through "upload mcu config".
 * The MCU rejects cmd 0xd1 (TLS trigger) until this whole sequence has run. */
static gboolean
gdix51c0_init_sequence (FpiDeviceGdix51c0 *self, GError **error)
{
  Gdix51c0Bus bus = {
    .dev = FP_DEVICE (self), .spi_fd = self->spi_fd,
    .irq_req = self->irq_req, .irq_offset = self->irq_offset,
    .irq_events = self->irq_events,
  };

  /* "required for ..." prefix: 01 05 00 00 00 00 00 88 — no ack.
   * Sent once before get_evk_version and once before get_mcu_state. */
  static const guint8 prefix01[] = { 0x01, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x88 };

  /* get_evk_version: prefix → 50ms → cmd 0xa8 (ack+resp same cycle) */
  if (!gdix51c0_cmd_no_ack (&bus, prefix01, sizeof (prefix01), "evk-prefix", error))
    return FALSE;
  g_usleep (50000);
  static const guint8 a8_payload[] = { 0xa8, 0x03, 0x00, 0x00, 0x00, 0xff };
  if (!gdix51c0_cmd_ack_optional_resp (&bus, a8_payload, sizeof (a8_payload),
                           200000, "get-evk-version", error))
    return FALSE;

  /* get_mcu_state: prefix → 50ms → cmd 0xaf with timestamp + ts-checksum
   * (single response, no ack — only one IRQ cycle).  We can reuse cmd_resp-
   * style flow: write, IRQ rise, read, IRQ fall.  No ack-only read.        */
  if (!gdix51c0_cmd_no_ack (&bus, prefix01, sizeof (prefix01), "mcu-state-prefix", error))
    return FALSE;
  g_usleep (50000);
  {
    GDateTime *now = g_date_time_new_now_utc ();
    guint ms = g_date_time_get_seconds (now) * 1000.0;
    g_date_time_unref (now);
    guint16 ms16 = (guint16) (ms & 0xffff);

    guint8 af_data[7];
    af_data[0] = 0xaf;
    af_data[1] = 0x06; af_data[2] = 0x00;        /* inner len */
    af_data[3] = 0x55;
    af_data[4] = ms16 & 0xff; af_data[5] = (ms16 >> 8) & 0xff;
    af_data[6] = 0x00;
    guint8 csum = gdix51c0_payload_checksum_ts (af_data, sizeof (af_data));
    guint8 af_full[9];
    memcpy (af_full, af_data, sizeof (af_data));
    af_full[7] = 0x00;        /* python: af 06 00 55 ms_lo ms_hi 00 00 csum */
    af_full[8] = csum;

    gsize mcu_state_len = 0;
    g_autofree guint8 *mcu_state = NULL;

    mcu_state = gdix51c0_cmd_single_resp_level (&bus,
                                                af_full,
                                                sizeof (af_full),
                                                GDIX51C0_FDT_TIMEOUT_USEC,
                                                &mcu_state_len,
                                                "get-mcu-state",
                                                error);
    if (!mcu_state)
      return FALSE;

    gdix51c0_debug_mcu_state_resp (mcu_state, mcu_state_len);
  }

  /* reset_sensor (cmd 0xa2) — ack then response in separate IRQ cycles */
  static const guint8 a2_payload[] = { 0xa2, 0x03, 0x00, 0x01, 0x14, 0xf0 };
  if (!gdix51c0_cmd_ack_then_resp (&bus, a2_payload, sizeof (a2_payload),
                                "reset-sensor-1", error))
    return FALSE;

  /* get MILAN_CHIPID (cmd 0x82) — ack+resp same cycle */
  static const guint8 chip_payload[] = { 0x82, 0x06, 0x00, 0x00, 0x00, 0x00, 0x04, 0x00, 0x1e };
  if (!gdix51c0_cmd_ack_optional_resp (&bus, chip_payload, sizeof (chip_payload),
                           200000, "milan-chipid", error))
    return FALSE;

  /* get OTP (cmd 0xa6) — ack then response in separate IRQ cycles */
  static const guint8 otp_payload[] = { 0xa6, 0x03, 0x00, 0x00, 0x00, 0x01 };
  if (!gdix51c0_cmd_ack_then_resp (&bus, otp_payload, sizeof (otp_payload),
                                "get-otp", error))
    return FALSE;

  /* reset_sensor again */
  if (!gdix51c0_cmd_ack_then_resp (&bus, a2_payload, sizeof (a2_payload),
                                "reset-sensor-2", error))
    return FALSE;

  /* setmode idle (cmd 0x70) — ack only, no separate response */
  static const guint8 setmode_idle[] = { 0x70, 0x03, 0x00, 0x14, 0x00, 0x23 };
  if (!gdix51c0_cmd_ack (&bus, setmode_idle, sizeof (setmode_idle),
                      "setmode-idle", error))
    return FALSE;

  /* send Dac 0x380bb500b300b300 (cmd 0x98) — ack+resp same cycle */
  static const guint8 dac_payload[] = {
    0x98, 0x09, 0x00, 0x38, 0x0b, 0xb5, 0x00, 0xb3, 0x00, 0xb3, 0x00, 0xab
  };
  if (!gdix51c0_cmd_ack_optional_resp (&bus, dac_payload, sizeof (dac_payload),
                           150000, "send-dac", error))
    return FALSE;

  /* upload mcu config (cmd 0x90) — 232-byte payload, ack+resp same cycle */
  static const guint8 cfg_payload[] = {
    0x90, 0xE1, 0x00, 0x70, 0x11, 0x74, 0x85, 0x00, 0x85, 0x2C, 0xB1, 0x18, 0xC9, 0x14, 0xDD, 0x00,
    0xDD, 0x00, 0xDD, 0x00, 0xBA, 0x00, 0x01, 0x80, 0xCA, 0x00, 0x04, 0x00, 0x84, 0x00, 0x15, 0xB3,
    0x86, 0x00, 0x00, 0xC4, 0x88, 0x00, 0x00, 0xBA, 0x8A, 0x00, 0x00, 0xB2, 0x8C, 0x00, 0x00, 0xAA,
    0x8E, 0x00, 0x00, 0xC1, 0x90, 0x00, 0xBB, 0xBB, 0x92, 0x00, 0xB1, 0xB1, 0x94, 0x00, 0x00, 0xA8,
    0x96, 0x00, 0x00, 0xB6, 0x98, 0x00, 0x00, 0x00, 0x9A, 0x00, 0x00, 0x00, 0xD2, 0x00, 0x00, 0x00,
    0xD4, 0x00, 0x00, 0x00, 0xD6, 0x00, 0x00, 0x00, 0xD8, 0x00, 0x00, 0x00, 0x50, 0x00, 0x01, 0x05,
    0xD0, 0x00, 0x00, 0x00, 0x70, 0x00, 0x00, 0x00, 0x72, 0x00, 0x78, 0x56, 0x74, 0x00, 0x34, 0x12,
    0x20, 0x00, 0x10, 0x40, 0x5C, 0x00, 0x00, 0x01, 0x20, 0x02, 0x38, 0x0B, 0x36, 0x02, 0xB5, 0x00,
    0x38, 0x02, 0xB3, 0x00, 0x3A, 0x02, 0xB3, 0x00, 0x2A, 0x01, 0x82, 0x03, 0x22, 0x00, 0x01, 0x20,
    0x24, 0x00, 0x14, 0x00, 0x80, 0x00, 0x01, 0x00, 0x5C, 0x00, 0x00, 0x01, 0x56, 0x00, 0x04, 0x20,
    0x58, 0x00, 0x03, 0x02, 0x32, 0x00, 0x0C, 0x02, 0x66, 0x00, 0x03, 0x00, 0x7C, 0x00, 0x00, 0x58,
    0x82, 0x00, 0x80, 0x1B, 0x2A, 0x01, 0x08, 0x00, 0x54, 0x00, 0x10, 0x01, 0x62, 0x00, 0x04, 0x03,
    0x64, 0x00, 0x19, 0x00, 0x66, 0x00, 0x03, 0x00, 0x7C, 0x00, 0x00, 0x58, 0x2A, 0x01, 0x08, 0x00,
    0x52, 0x00, 0x08, 0x00, 0x54, 0x00, 0x00, 0x01, 0x66, 0x00, 0x03, 0x00, 0x7C, 0x00, 0x00, 0x58,
    0x00, 0x53, 0x66, 0x8F
  };
  if (!gdix51c0_cmd_ack_optional_resp (&bus, cfg_payload, sizeof (cfg_payload),
                                    150000, "upload-mcu-config", error))
    return FALSE;
  
  g_usleep (200000);
  gdix51c0_irq_drain (self->irq_req, self->irq_events);

  fp_dbg ("gdix51c0: pre-TLS init sequence complete");
  return TRUE;
}

static gboolean
gdix51c0_load_psk (guint8 out[32], GError **err)
{
  const char *hex = g_getenv (GDIX51C0_ENV_PSK_HEX);
  if (!hex || !*hex)
    {
      g_set_error (err, G_IO_ERROR, G_IO_ERROR_NOT_INITIALIZED,
                   "gdix51c0: $%s not set; full activation needs the 32-byte PSK in hex",
                   GDIX51C0_ENV_PSK_HEX);
      return FALSE;
    }
  if (strlen (hex) != 64)
    {
      g_set_error (err, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                   "gdix51c0: $%s must be 64 hex chars (got %zu)",
                   GDIX51C0_ENV_PSK_HEX, strlen (hex));
      return FALSE;
    }
  for (int i = 0; i < 32; i++)
    {
      char b[3] = { hex[2*i], hex[2*i + 1], 0 };
      gchar *endp = NULL;
      out[i] = (guint8) g_ascii_strtoull (b, &endp, 16);
      if (!endp || *endp != 0)
        {
          g_set_error_literal (err, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                               "gdix51c0: bad hex in PSK env");
          return FALSE;
        }
    }
  return TRUE;
}

static void gdix51c0_session_deactivate (FpiDeviceGdix51c0 *self);

static gboolean
gdix51c0_session_activate_once (FpiDeviceGdix51c0 *self, GError **error)
{
  guint8 psk[32];

  G_DEBUG_HERE ();

  self->session_desynced = FALSE;
  self->fdt_down_sample_valid = FALSE;
  memcpy (self->fdt_down_regs,
          gdix51c0_default_fdt_down_regs,
          sizeof (self->fdt_down_regs));

  if (!gdix51c0_boot_probe (self, error))
    return FALSE;

  if (!gdix51c0_init_sequence (self, error))
    return FALSE;

  if (!gdix51c0_load_psk (psk, error))
    return FALSE;

  if (!gdix51c0_tls_init (&self->tls, FP_DEVICE (self), self->spi_fd,
                       self->irq_req, self->irq_events, self->irq_offset,
                       psk, sizeof (psk), error))
    return FALSE;

  if (!gdix51c0_tls_handshake (&self->tls, error))
    {
      gdix51c0_tls_free (&self->tls);
      return FALSE;
    }
  self->tls_ready = TRUE;

  /* Post-TLS init: cmd 0xd4 (ack-only). */
  Gdix51c0Bus bus = {
    .dev = FP_DEVICE (self), .spi_fd = self->spi_fd,
    .irq_req = self->irq_req, .irq_offset = self->irq_offset,
    .irq_events = self->irq_events,
  };
  g_usleep (100000);  /* python: manual_sleep(0.1) before cmd 0xd4 */

  static const guint8 d4_payload[] = {
    0xd4, 0x03, 0x00, 0x00, 0x00, 0xd3
  };

  {
    g_autoptr(GError) d4_error = NULL;

    if (!gdix51c0_cmd_ack_optional_resp (&bus,
                                        d4_payload,
                                        sizeof (d4_payload),
                                        250000,
                                        "post-tls-d4",
                                        &d4_error))
      {
        g_set_error (error,
                   G_IO_ERROR,
                   G_IO_ERROR_CONNECTION_CLOSED,
                   "gdix51c0: post-tls-d4 failed after TLS: %s",
                   d4_error ? d4_error->message : "?");
        return FALSE;
      }
  }

  fp_dbg ("gdix51c0: ready for capture (TLS done)");
  return TRUE;
}

static gboolean
gdix51c0_session_activate (FpiDeviceGdix51c0 *self, GError **error)
{
  for (guint attempt = 0; attempt < 4; attempt++)
    {
      g_autoptr(GError) local_error = NULL;

      if (gdix51c0_session_activate_once (self, &local_error))
        return TRUE;

      gdix51c0_session_deactivate (self);

      if (attempt == 3)
        {
          g_propagate_error (error, g_steal_pointer (&local_error));
          return FALSE;
        }

      fp_warn ("gdix51c0: activation failed on attempt %u/4 (%s); resetting and retrying",
               attempt + 1,
               local_error ? local_error->message : "?");

      g_usleep (1000000);
    }

  g_set_error_literal (error,
                       G_IO_ERROR,
                       G_IO_ERROR_FAILED,
                       "gdix51c0: activation failed unexpectedly");
  return FALSE;
}

static void
gdix51c0_session_deactivate (FpiDeviceGdix51c0 *self)
{
  G_DEBUG_HERE ();

  if (self->tls_ready)
    {
      gdix51c0_tls_free (&self->tls);
      self->tls_ready = FALSE;
    }

  self->session_desynced = FALSE;

  gdix51c0_irq_drain (self->irq_req, self->irq_events);

  if (self->gpio_chip)
    {
      g_autoptr(GError) error = NULL;

      if (!gdix51c0_reset_pulse (self->gpio_chip,
                                 self->reset_offset,
                                 &error))
        {
          fp_dbg ("gdix51c0: reset during session deactivate failed: %s",
                  error ? error->message : "?");
        }

      /*
       * Important: 250ms is sometimes not enough. Logs show the next
       * activation can miss get-mcu-state or TLS ClientHello.
       */
      g_usleep (1000000);

      gdix51c0_irq_drain (self->irq_req, self->irq_events);
    }
}


/* ------------------------------------------------------------------ */
/* Image decode: 6 packed bytes -> 4 12-bit samples (kept as guint16   */
/* for downstream diff/normalization).  Mirrors image_decode.py.       */
/* ------------------------------------------------------------------ */
static void
gdix51c0_decode_12bpp_to_16bpp (const guint8 *packed, gsize packed_len,
                             guint16 *out16, gsize out_pixels)
{
  /* 6 packed bytes encode 4 12-bit samples (per python image_decode):
   *   o1 = ((c[0] & 0xf) << 8) | c[1]
   *   o2 = (c[3] << 4)         | (c[0] >> 4)
   *   o3 = ((c[5] & 0xf) << 8) | c[2]
   *   o4 = (c[4] << 4)         | (c[5] >> 4)
   */
  gsize chunks = MIN (packed_len / 6, out_pixels / 4);
  for (gsize i = 0; i < chunks; i++)
    {
      const guint8 *c = packed + i * 6;
      out16[i*4 + 0] = (((guint16) (c[0] & 0x0f)) << 8) | c[1];
      out16[i*4 + 1] = (((guint16) c[3])         << 4) | (c[0] >> 4);
      out16[i*4 + 2] = (((guint16) (c[5] & 0x0f)) << 8) | c[2];
      out16[i*4 + 3] = (((guint16) c[4])         << 4) | (c[5] >> 4);
    }
}

static gboolean
gdix51c0_wait_irq_level (FpiDeviceGdix51c0 *self,
                         gboolean           high,
                         guint              timeout_usec,
                         const char        *label,
                         GError           **error)
{
  enum gpiod_line_value value =
    gpiod_line_request_get_value (self->irq_req, self->irq_offset);

  if (value < 0)
    {
      g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: %s: get IRQ value failed", label);
      return FALSE;
    }
  if ((value == GPIOD_LINE_VALUE_ACTIVE) == high)
    return TRUE;

  return gdix51c0_irq_wait (self->irq_req, self->irq_events, high,
                            self->irq_offset, timeout_usec, label, error);
}

/* ------------------------------------------------------------------ */
/* One round of FDT-down (cmd 0x32): write setmode, read ack, read     */
/* response.  Sets *touchflag to response byte [5] (0x3f when finger    */
/* present, 0x00 when not).  Used by the finger-detection poll.         */
/* ------------------------------------------------------------------ */
static gboolean
gdix51c0_fdt_down_round (FpiDeviceGdix51c0 *self,
                         guint              finger_timeout_usec,
                         guint8            *touchflag,
                         gboolean          *retry,
                         GError           **error)
{
  guint8 fdt_data[2 + 12 + 2];
  gsize fdt_pkt_len = 0;
  g_autofree guint8 *fdt_packet = NULL;
  g_autoptr(GError) local_error = NULL;

  *touchflag = 0;
  *retry = FALSE;

  fdt_data[0] = 0x08;
  fdt_data[1] = 0x01;
  memcpy (&fdt_data[2], self->fdt_down_regs, sizeof (self->fdt_down_regs));

  /* Do not use a constant timestamp. */
  guint16 ts = (guint16) (g_get_monotonic_time () & 0xffff);
  fdt_data[14] = ts & 0xff;
  fdt_data[15] = ts >> 8;

  fdt_packet =
    gdix51c0_make_payload_packet (GDIX51C0_CMD_FDT_DOWN,
                                  fdt_data, sizeof (fdt_data),
                                  &fdt_pkt_len);

  g_usleep (80000);

  if (!gdix51c0_wait_irq_level (self, FALSE, GDIX51C0_FDT_ACK_TIMEOUT_USEC,
                                "fdt-pre-idle", &local_error))
    {
      *retry = TRUE;
      return TRUE;
    }

  gdix51c0_irq_drain (self->irq_req, self->irq_events);

  if (!gdix51c0_spi_write (FP_DEVICE (self), self->spi_fd, GDIX51C0_PKT_WRITE,
                           fdt_packet, fdt_pkt_len, error))
    return FALSE;

  if (!gdix51c0_irq_wait (self->irq_req, self->irq_events, TRUE,
                          self->irq_offset, GDIX51C0_FDT_ACK_TIMEOUT_USEC,
                          "fdt-down-ack-rise", &local_error))
    {
      fp_dbg ("gdix51c0: FDT arm got no ACK; retrying");
      *retry = TRUE;
      return TRUE;
    }

  {
    gsize n = 0;
    g_autofree guint8 *ack =
      gdix51c0_spi_read (FP_DEVICE (self), self->spi_fd, &n, &local_error);

    if (!ack)
      {
        fp_dbg ("gdix51c0: FDT ACK read failed: %s; retrying",
                local_error ? local_error->message : "?");
        *retry = TRUE;
        return TRUE;
      }
  }

  if (!gdix51c0_irq_wait (self->irq_req, self->irq_events, FALSE,
                          self->irq_offset, GDIX51C0_FDT_ACK_TIMEOUT_USEC,
                          "fdt-down-ack-fall", &local_error))
    {
      *retry = TRUE;
      return TRUE;
    }

  if (!gdix51c0_irq_wait (self->irq_req, self->irq_events, TRUE,
                          self->irq_offset, finger_timeout_usec,
                          "fdt-down-finger-rise", error))
    return FALSE;

  {
    gsize n = 0;
    g_autofree guint8 *fdt_resp =
      gdix51c0_spi_read (FP_DEVICE (self), self->spi_fd, &n, &local_error);

    if (!fdt_resp)
      {
        fp_dbg ("gdix51c0: FDT response read failed: %s; retrying",
                local_error ? local_error->message : "?");
        *retry = TRUE;
        return TRUE;
      }

    {
      guint16 inner_len = n >= 3 ? ((guint16) fdt_resp[1] | ((guint16) fdt_resp[2] << 8)) : 0;
      gboolean checksum_ok = n > 0 && gdix51c0_payload_checksum (fdt_resp, n - 1) == fdt_resp[n - 1];
      GString *s = g_string_sized_new (n * 3 + 1);

      for (gsize i = 0; i < n; i++)
        g_string_append_printf (s, "%02x%s", fdt_resp[i], i + 1 < n ? " " : "");

      fp_dbg ("gdix51c0: FDT response len=%zu cmd=0x%02x inner_len=%u checksum=%s data=%s",
              n,
              n > 0 ? fdt_resp[0] : 0,
              inner_len,
              checksum_ok ? "ok" : "bad",
              s->str);
      g_string_free (s, TRUE);
    }

    if (n < 17 || fdt_resp[0] != GDIX51C0_CMD_FDT_DOWN)
      {
        fp_dbg ("gdix51c0: ignoring non-FDT-down response while waiting for finger: "
                "len=%zu cmd=0x%02x tail=0x%02x",
                n,
                n > 0 ? fdt_resp[0] : 0,
                n > 0 ? fdt_resp[n - 1] : 0);
        *touchflag = GDIX51C0_TOUCH_IGNORE;
        return TRUE;
      }

    *touchflag = (n > 5) ? fdt_resp[5] : 0;

    self->fdt_down_sample_valid = FALSE;
    if (n >= 20)
      {
        for (guint i = 0; i < 6; i++)
          self->fdt_down_sample[i] = (guint16) fdt_resp[7 + i * 2] |
                                     ((guint16) fdt_resp[8 + i * 2] << 8);

        self->fdt_down_sample_valid = TRUE;
      }

    /* Windows derives the next FDT-down base from an FDT-up response.  Do not
     * retune FDT-down thresholds from zero-touch FDT-down packets; doing so can
     * make subsequent finger-down events never reach touchflag 0x3f. */
  }

  if (!gdix51c0_irq_wait (self->irq_req, self->irq_events, FALSE,
                          self->irq_offset, GDIX51C0_FDT_ACK_TIMEOUT_USEC,
                          "fdt-down-finger-fall", &local_error))
    {
      *retry = TRUE;
      return TRUE;
    }

  return TRUE;
}

static gboolean
gdix51c0_wait_for_finger (FpiDeviceGdix51c0 *self, GError **error)
{
  gint64 finger_deadline = g_get_monotonic_time () +
                           (gint64) GDIX51C0_FINGER_TIMEOUT_USEC;
  guint zero_touch_count = 0;
  guint retry_count = 0;
  guint nonready_count = 0;
  guint16 zero_base[6] = { 0 };
  gboolean zero_base_valid = FALSE;

  fp_dbg ("gdix51c0: arming FDT-down and waiting for finger GPIO IRQ...");

  for (;;)
    {
      guint8 touchflag = 0;
      gboolean retry = FALSE;
      gint64 now = g_get_monotonic_time ();
      guint remaining_usec;

      if (now >= finger_deadline)
        {
          g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                               "gdix51c0: no finger detected within timeout");
          return FALSE;
        }

      remaining_usec = (guint) MIN (finger_deadline - now,
                                    (gint64) G_MAXUINT);

      if (!gdix51c0_irq_wait (self->irq_req, self->irq_events, FALSE,
                              self->irq_offset, GDIX51C0_FDT_TIMEOUT_USEC,
                              "wait-finger-pre-idle", error))
        return FALSE;

      gdix51c0_irq_drain (self->irq_req, self->irq_events);

      if (!gdix51c0_fdt_down_round (self, remaining_usec,
                                    &touchflag, &retry, error))
        return FALSE;

      if (retry)
        {
          retry_count++;

          fp_dbg ("gdix51c0: FDT retry %u; backing off", retry_count);

          g_usleep (350000);
          continue;
        }

      retry_count = 0;

      if (touchflag == 0x3f)
        {
          fp_dbg ("gdix51c0: finger detected, touchflag=0x%02x", touchflag);
          return TRUE;
        }

      if (touchflag == GDIX51C0_TOUCH_IGNORE)
        {
          fp_dbg ("gdix51c0: ignored async/non-FDT packet while waiting for finger");
          g_usleep (50000);
          continue;
        }

      if (touchflag == 0x00)
        {
          zero_touch_count++;
          nonready_count = 0;

          if (self->fdt_down_sample_valid)
            {
              guint total_drop = 0;
              guint max_drop = 0;

              if (!zero_base_valid)
                {
                  memcpy (zero_base, self->fdt_down_sample, sizeof (zero_base));
                  zero_base_valid = TRUE;
                }
              else
                {
                  for (guint i = 0; i < 6; i++)
                    {
                      guint drop = zero_base[i] > self->fdt_down_sample[i] ?
                                   zero_base[i] - self->fdt_down_sample[i] : 0;

                      total_drop += drop;
                      max_drop = MAX (max_drop, drop);
                    }

                  fp_dbg ("gdix51c0: zero-touch FDT base delta total=%u max=%u",
                          total_drop, max_drop);

                  if (max_drop >= GDIX51C0_FDT_HOST_TOUCH_MIN_DROP &&
                      total_drop >= GDIX51C0_FDT_HOST_TOUCH_TOTAL_DROP)
                    {
                      fp_dbg ("gdix51c0: treating zero-touch FDT as finger by base delta");
                      return TRUE;
                    }
                }
            }

          fp_dbg ("gdix51c0: ignoring zero-touch FDT event %u; backing off",
                  zero_touch_count);

          if (zero_touch_count >= 3)
            {
              g_usleep (500000);
              zero_touch_count = 0;
            }
          else
            {
              g_usleep (200000);
            }

          continue;
        }

      /*
       * Non-zero but not 0x3f: contact-ish, but not known-good for image
       * capture. Do not start cmd 0x22 from this state.
       *
       * The UI may feel like it wants to keep scanning after no-match, but
       * for security we require a clean 0x3f placement.
       */
      nonready_count++;
      zero_touch_count = 0;

      fp_dbg ("gdix51c0: ignoring non-ready FDT touchflag=0x%02x event %u; require clean 0x3f",
              touchflag, nonready_count);

      g_usleep (300000);
    }
}

/* Wait for finger lift with short FDT-down probes.  touchflag returns 0 once
 * the finger is no longer touching.  Used between enroll stages so libfprint's
 * "place finger again" prompt is honoured instead of capturing the same frame
 * back-to-back. */
static gboolean
gdix51c0_wait_for_lift (FpiDeviceGdix51c0 *self, GError **error)
{
  fp_dbg ("gdix51c0: waiting for finger lift (short FDT-down probes)...");
  gint64 lift_deadline = g_get_monotonic_time () +
                          (gint64) GDIX51C0_FINGER_TIMEOUT_USEC;
  guint retry_count = 0;

  for (;;)
    {
      guint8 touchflag = 0xff;
      gboolean retry = FALSE;

      if (g_get_monotonic_time () >= lift_deadline)
        {
          g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                               "gdix51c0: finger not lifted within timeout");
          return FALSE;
        }

      if (!gdix51c0_fdt_down_round (self, GDIX51C0_FDT_TIMEOUT_USEC,
                                    &touchflag, &retry, error))
        return FALSE;

      if (retry)
        {
          retry_count++;
          fp_dbg ("gdix51c0: FDT lift retry %u; backing off", retry_count);
          g_usleep (120000);
          continue;
        }

      retry_count = 0;

      if (touchflag == 0)
        {
          fp_dbg ("gdix51c0: finger lifted");
          return TRUE;
        }

      g_usleep (100000);
    }
}

static void
gdix51c0_dump_debug_views (const guint16 *raw)
{
  if (!g_getenv ("GDIX51C0_DUMP_FRAMES"))
    return;

  guint16 mn = 0xffff, mx = 0;

  for (gsize i = 0; i < GDIX51C0_FRAME_PIXELS; i++)
    {
      if (raw[i] < mn)
        mn = raw[i];
      if (raw[i] > mx)
        mx = raw[i];
    }

  guint span = mx > mn ? (guint) (mx - mn) : 1;

  guint8 normal[GDIX51C0_FRAME_PIXELS];
  guint8 inverted[GDIX51C0_FRAME_PIXELS];

  for (gsize i = 0; i < GDIX51C0_FRAME_PIXELS; i++)
    {
      guint v = ((guint) (raw[i] - mn) * 255u + span / 2) / span;
      v = MIN (v, 255u);

      normal[i] = (guint8) v;
      inverted[i] = (guint8) (255u - v);
    }

  gint64 ts = g_get_monotonic_time ();

  g_autofree char *path_normal =
    g_strdup_printf ("/tmp/gdix51c0_debug_normal_%" G_GINT64_FORMAT ".pgm", ts);
  g_autofree char *path_inverted =
    g_strdup_printf ("/tmp/gdix51c0_debug_inverted_%" G_GINT64_FORMAT ".pgm", ts);

  FILE *fp = fopen (path_normal, "wb");
  if (fp)
    {
      fprintf (fp, "P5\n%d %d\n255\n",
               GDIX51C0_SENSOR_WIDTH,
               GDIX51C0_SENSOR_HEIGHT);
      fwrite (normal, 1, GDIX51C0_FRAME_PIXELS, fp);
      fclose (fp);
    }

  fp = fopen (path_inverted, "wb");
  if (fp)
    {
      fprintf (fp, "P5\n%d %d\n255\n",
               GDIX51C0_SENSOR_WIDTH,
               GDIX51C0_SENSOR_HEIGHT);
      fwrite (inverted, 1, GDIX51C0_FRAME_PIXELS, fp);
      fclose (fp);
    }

  fp_dbg ("gdix51c0: dumped debug views normal=%s inverted=%s raw_mn=%u raw_mx=%u",
          path_normal, path_inverted, mn, mx);
}

/* ------------------------------------------------------------------ */
/* Single image capture (cmd 0x22 -> ack -> read -> decrypt -> decode). */
/* Returns a fresh guint16[GDIX51C0_FRAME_PIXELS] on success.              */
/* ------------------------------------------------------------------ */
static guint16 *
gdix51c0_capture_image_raw (FpiDeviceGdix51c0 *self, GError **error)
{
  static const guint8 img_setmode[] = {
    0x22, 0x03, 0x00, 0x01, 0x00, 0x84
  };

#define GDIX51C0_IMAGE_PREFIX_LEN (3 + 5)

  /*
   * Retry is safe only before cmd 0x22 has been ACKed.
   *
   * After the MCU ACKs image setmode, it may emit exactly one TLS
   * application-data record. If we miss it and send another 0x22 inside
   * the same TLS session, OpenSSL's record sequence can diverge and
   * AES-GCM will fail with bad MAC / cipher operation failed.
   */
  for (guint attempt = 1; attempt <= 3; attempt++)
    {
      g_autoptr(GError) local_error = NULL;

      fp_dbg ("gdix51c0: image capture attempt %u", attempt);

      /* Windows requests the image roughly 10 ms after the FDT-down event. */
      g_usleep (10000);

      if (!gdix51c0_wait_irq_level (self, FALSE, GDIX51C0_IMAGE_ACK_TIMEOUT_USEC,
                                    "img-pre-idle", &local_error))
        {
          fp_dbg ("gdix51c0: img-pre-idle failed: %s",
                  local_error ? local_error->message : "?");
          continue;
        }

      gdix51c0_irq_drain (self->irq_req, self->irq_events);

      if (!gdix51c0_spi_write (FP_DEVICE (self), self->spi_fd,
                               GDIX51C0_PKT_WRITE,
                               img_setmode, sizeof (img_setmode),
                               &local_error))
        {
          g_propagate_error (error, g_steal_pointer (&local_error));
          return NULL;
        }

      if (!gdix51c0_irq_wait (self->irq_req, self->irq_events, TRUE,
                              self->irq_offset, GDIX51C0_IMAGE_ACK_TIMEOUT_USEC,
                              "img-ack-rise", &local_error))
        {
          fp_dbg ("gdix51c0: img-setmode got no ACK: %s; retrying",
                  local_error ? local_error->message : "?");
          continue;
        }

      {
        gsize n = 0;
        g_autofree guint8 *ack =
          gdix51c0_spi_read (FP_DEVICE (self), self->spi_fd, &n,
                             &local_error);

        if (!ack)
          {
            fp_dbg ("gdix51c0: img ACK read failed before TLS image stream: %s; retrying",
                    local_error ? local_error->message : "?");
            continue;
          }

        fp_dbg ("gdix51c0: img-setmode ACK read %zu B", n);
      }

      /*
       * From here onward, do not continue the loop on failure.
       * The image TLS record may already have been produced by the MCU.
       */
      if (!gdix51c0_irq_wait (self->irq_req, self->irq_events, FALSE,
                              self->irq_offset, GDIX51C0_IMAGE_ACK_TIMEOUT_USEC,
                              "img-ack-fall", &local_error))
        {
          g_set_error (error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                       "gdix51c0: image ACK fall failed after setmode ACK: %s",
                       local_error ? local_error->message : "?");
          return NULL;
        }

      if (!gdix51c0_irq_wait (self->irq_req, self->irq_events, TRUE,
                        self->irq_offset, GDIX51C0_IMAGE_TIMEOUT_USEC,
                        "img-rise", &local_error))
        {
          g_set_error (error,
             G_IO_ERROR,
             G_IO_ERROR_CONNECTION_CLOSED,
             "gdix51c0: image data IRQ did not arrive after setmode ACK: %s",
             local_error ? local_error->message : "?");
          return NULL;
        }

      gsize record_len = 0;
      g_autofree guint8 *encrypted =
        gdix51c0_spi_read (FP_DEVICE (self), self->spi_fd,
                           &record_len, &local_error);

      if (!encrypted)
        {
          g_set_error (error,
                      G_IO_ERROR,
                      G_IO_ERROR_CONNECTION_CLOSED,
                      "gdix51c0: image read failed after setmode ACK: %s",
                      local_error ? local_error->message : "?");
          return NULL;
        }

      if (!gdix51c0_irq_wait (self->irq_req, self->irq_events, FALSE,
                              self->irq_offset, GDIX51C0_IMAGE_TIMEOUT_USEC,
                              "img-fall", &local_error))
        {
          g_set_error (error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                       "gdix51c0: image fall failed after image read: %s",
                       local_error ? local_error->message : "?");
          return NULL;
        }

      if (record_len >= 5)
        {
          fp_dbg ("gdix51c0: image TLS candidate len=%zu hdr=%02x %02x %02x %02x %02x",
                  record_len,
                  encrypted[0], encrypted[1], encrypted[2],
                  encrypted[3], encrypted[4]);
        }
      else
        {
          g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                       "gdix51c0: image packet too short: %zu", record_len);
          return NULL;
        }

      if (!gdix51c0_tls_record_header_ok (encrypted, record_len))
        {
          g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                       "gdix51c0: image packet is not a valid TLS record "
                       "len=%zu hdr=%02x %02x %02x %02x %02x",
                       record_len,
                       encrypted[0], encrypted[1], encrypted[2],
                       encrypted[3], encrypted[4]);
          return NULL;
        }

      if (encrypted[0] != 0x17)
        {
          g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                       "gdix51c0: image packet is TLS type 0x%02x, expected appdata 0x17",
                       encrypted[0]);
          return NULL;
        }

      gsize plain_len = 0;
      g_autofree guint8 *plain =
        gdix51c0_tls_decrypt_record (&self->tls,
                                     encrypted, record_len,
                                     &plain_len, &local_error);

      if (!plain)
        {
          g_set_error (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                       "gdix51c0: image TLS decrypt failed; session is no longer reusable: %s",
                       local_error ? local_error->message : "?");
          return NULL;
        }

      fp_dbg ("gdix51c0: image plaintext len=%zu", plain_len);

      if (plain_len < GDIX51C0_IMAGE_PREFIX_LEN + GDIX51C0_FRAME_BYTES)
        {
          g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                       "gdix51c0: image plaintext too short: %zu, need at least %u",
                       plain_len,
                       (guint) (GDIX51C0_IMAGE_PREFIX_LEN + GDIX51C0_FRAME_BYTES));
          return NULL;
        }

      const guint8 *image_packed = plain + GDIX51C0_IMAGE_PREFIX_LEN;

      guint16 *raw = g_malloc (GDIX51C0_FRAME_PIXELS * sizeof (guint16));
      gdix51c0_decode_12bpp_to_16bpp (image_packed,
                                      GDIX51C0_FRAME_BYTES,
                                      raw,
                                      GDIX51C0_FRAME_PIXELS);
      return raw;
    }

  g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                       "gdix51c0: image setmode failed before ACK after retries");
  return NULL;

#undef GDIX51C0_IMAGE_PREFIX_LEN
}

static gboolean
gdix51c0_capture_error_needs_session_restart (const GError *error)
{
  if (!error)
    return FALSE;

  if (g_error_matches (error, G_IO_ERROR, G_IO_ERROR_CONNECTION_CLOSED))
    return TRUE;

  /*
   * After image setmode ACK, an all-FF/all-zero image read means the
   * current TLS/session is not trustworthy anymore.
   */
  if (error->message &&
      (strstr (error->message, "image read failed after setmode ACK") ||
       strstr (error->message, "image data IRQ did not arrive after setmode ACK") ||
       strstr (error->message, "FDT lost sync")))
    return TRUE;

  return FALSE;
}

static int
gdix51c0_u16_compare (gconstpointer a, gconstpointer b)
{
  guint16 va = *(const guint16 *) a;
  guint16 vb = *(const guint16 *) b;
  return (va > vb) - (va < vb);
}

/* ------------------------------------------------------------------ */
/* Top-level capture: wait for finger, capture one native 64x80 frame, */
/* contrast-normalize it, and return the 8-bit frame for SIGFM.         */
/* ------------------------------------------------------------------ */
static guint8 *
gdix51c0_capture_one_raw8 (FpiDeviceGdix51c0 *self, GError **error)
{
  for (guint session_attempt = 0; session_attempt < 2; session_attempt++)
    {
      g_autoptr(GError) local_error = NULL;
      g_autofree guint16 *raw = NULL;

      if (!gdix51c0_wait_for_finger (self, &local_error))
        {
          if (gdix51c0_capture_error_needs_session_restart (local_error) &&
              session_attempt == 0)
            {
              fp_warn ("gdix51c0: capture wait lost sync; restarting session: %s",
                       local_error ? local_error->message : "?");

              gdix51c0_session_deactivate (self);

              if (!gdix51c0_session_activate (self, &local_error))
                {
                  g_propagate_error (error, g_steal_pointer (&local_error));
                  return NULL;
                }

              fpi_device_report_finger_status (FP_DEVICE (self),
                                               FP_FINGER_STATUS_NEEDED);
              continue;
            }

          g_propagate_error (error, g_steal_pointer (&local_error));
          return NULL;
        }

      fpi_device_report_finger_status (FP_DEVICE (self),
                                       FP_FINGER_STATUS_PRESENT);

      raw = gdix51c0_capture_image_raw (self, &local_error);
      if (!raw)
        {
          if (gdix51c0_capture_error_needs_session_restart (local_error) &&
              session_attempt == 0)
            {
              fp_warn ("gdix51c0: image capture lost sync; restarting session: %s",
                       local_error ? local_error->message : "?");

              gdix51c0_session_deactivate (self);

              if (!gdix51c0_session_activate (self, &local_error))
                {
                  g_propagate_error (error, g_steal_pointer (&local_error));
                  return NULL;
                }

              fpi_device_report_finger_status (FP_DEVICE (self),
                                               FP_FINGER_STATUS_NEEDED);
              continue;
            }

          g_propagate_error (error, g_steal_pointer (&local_error));
          return NULL;
        }

      gdix51c0_dump_debug_views (raw);

      /* Percentile stretch instead of raw min/max: a few hot/dead pixels can
       * otherwise consume most of the output range and make ridges look grey. */
      g_autofree guint16 *sorted =
        g_memdup2 (raw, GDIX51C0_FRAME_PIXELS * sizeof (guint16));
      qsort (sorted,
             GDIX51C0_FRAME_PIXELS,
             sizeof (guint16),
             gdix51c0_u16_compare);

      guint16 mn = sorted[(GDIX51C0_FRAME_PIXELS * 2) / 100];
      guint16 mx = sorted[(GDIX51C0_FRAME_PIXELS * 98) / 100];

      guint span = mx > mn ? (guint) (mx - mn) : 1;
      guint8 small_px[GDIX51C0_FRAME_PIXELS];

      for (gsize i = 0; i < GDIX51C0_FRAME_PIXELS; i++)
        {
          guint16 clipped = CLAMP (raw[i], mn, mx);
          guint v = ((guint) (clipped - mn) * 255u + span / 2) / span;
          small_px[i] = (guint8) MIN (v, 255u);
        }

      fp_dbg ("gdix51c0: percentile stretch p02=%u p98=%u span=%u",
              mn, mx, span);

      if (g_getenv ("GDIX51C0_DUMP_FRAMES"))
        {
          /* Optional side-dump for visual sanity checks. */
          g_autofree char *path =
            g_strdup_printf ("/tmp/gdix51c0_frame_%" G_GINT64_FORMAT ".pgm",
                             g_get_monotonic_time ());

          FILE *fp = fopen (path, "wb");
          if (fp)
            {
              fprintf (fp, "P5\n%d %d\n255\n",
                       GDIX51C0_SENSOR_WIDTH,
                       GDIX51C0_SENSOR_HEIGHT);
              fwrite (small_px, 1, GDIX51C0_FRAME_PIXELS, fp);
              fclose (fp);

              fp_dbg ("gdix51c0: dumped frame %s", path);
            }
        }

      return g_memdup2 (small_px, GDIX51C0_FRAME_PIXELS);
    }

  g_set_error_literal (error,
                       G_IO_ERROR,
                       G_IO_ERROR_FAILED,
                       "gdix51c0: capture failed after session restart");
  return NULL;
}

static void
gdix51c0_wait_for_lift_report (FpiDeviceGdix51c0 *self)
{
  GError *lift_err = NULL;

  if (!gdix51c0_wait_for_lift (self, &lift_err))
    {
      fp_warn ("gdix51c0: lift wait failed (%s), reporting off anyway",
               lift_err ? lift_err->message : "?");
      g_clear_error (&lift_err);
    }
  fpi_device_report_finger_status (FP_DEVICE (self), FP_FINGER_STATUS_NONE);
}

static gint
gdix51c0_match_print (SigfmImgInfo *probe_info,
                      FpPrint      *print,
                      gint         *out_weak_samples,
                      gint         *out_medium_samples,
                      GError      **error)
{
  g_autoptr(GVariant) data = NULL;
  GVariantIter iter;
  GVariant *child;
  gint best_score = 0;
  gint weak_samples = 0;
  gint medium_samples = 0;
  gint sample_idx = 0;

  g_object_get (G_OBJECT (print), "fpi-data", &data, NULL);
  if (!data || !g_variant_is_of_type (data, G_VARIANT_TYPE ("aay")))
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                           "gdix51c0: enrolled print is not SIGFM raw image data");
      return -1;
    }

  g_variant_iter_init (&iter, data);
  while ((child = g_variant_iter_next_value (&iter)))
    {
      gsize len = 0;
      const guint8 *image = g_variant_get_fixed_array (child, &len, 1);

      if (len == GDIX51C0_FRAME_PIXELS)
        {
          SigfmImgInfo *template_info =
            sigfm_extract (image,
                           GDIX51C0_SENSOR_WIDTH,
                           GDIX51C0_SENSOR_HEIGHT);

          gint template_kp = sigfm_keypoints_count (template_info);

          if (template_kp < GDIX51C0_SIGFM_TEMPLATE_KP_MIN)
            {
              fp_dbg ("gdix51c0: sample %d skipped: low template keypoints=%d",
                      sample_idx, template_kp);
              sigfm_free_info (template_info);
              sample_idx++;
              g_variant_unref (child);
              continue;
            }

          gint score = sigfm_match_score (probe_info, template_info);

          fp_dbg ("gdix51c0: sample %d SIGFM keypoints=%d score=%d",
                  sample_idx, template_kp, score);

          sigfm_free_info (template_info);

          if (score >= GDIX51C0_SIGFM_SCORE_WEAK)
            weak_samples++;

          if (score >= GDIX51C0_SIGFM_SCORE_MEDIUM)
            medium_samples++;

          if (score > best_score)
            best_score = score;
        }
      else
        {
          fp_dbg ("gdix51c0: sample %d skipped: invalid raw len=%zu",
                  sample_idx, len);
        }

      sample_idx++;
      g_variant_unref (child);
    }

  if (out_weak_samples)
    *out_weak_samples = weak_samples;

  if (out_medium_samples)
    *out_medium_samples = medium_samples;

  fp_dbg ("gdix51c0: match summary: best=%d weak_samples=%d medium_samples=%d",
          best_score, weak_samples, medium_samples);

  return best_score;
}

static gboolean
gdix51c0_sigfm_is_match (gint best_score,
                         gint weak_samples,
                         gint medium_samples,
                         gint probe_kp)
{
  if (probe_kp < GDIX51C0_SIGFM_PROBE_KP_MIN)
    return FALSE;

  /* Obvious same-finger case. */
  if (best_score >= GDIX51C0_SIGFM_SCORE_VERY_STRONG)
    return TRUE;

  /* Strong single-template match, but not as loose as the old best>=80. */
  if (best_score >= GDIX51C0_SIGFM_SCORE_STRONG)
    return TRUE;

  /*
   * Moderate multi-sample agreement.
   * Keep this stricter than the earlier false-accept-ish case.
   */
  if (best_score >= GDIX51C0_SIGFM_SCORE_MEDIUM &&
      medium_samples >= 1 &&
      weak_samples >= 3)
    return TRUE;

  return FALSE;
}

static void
gdix51c0_enroll (FpDevice *dev)
{
  FpiDeviceGdix51c0 *self = FPI_DEVICE_GDIX51C0 (dev);
  g_autoptr(GPtrArray) images = NULL;
  GError *error = NULL;
  FpPrint *print = NULL;

  if (!gdix51c0_session_activate (self, &error))
    goto out;

  images = g_ptr_array_new_with_free_func (g_free);
  for (gint stage = 0; stage < GDIX51C0_ENROLL_SAMPLES; stage++)
    {
      guint8 *image;

      fpi_device_report_finger_status (dev, FP_FINGER_STATUS_NEEDED);
      image = gdix51c0_capture_one_raw8 (self, &error);
      if (!image)
        goto out;

      g_ptr_array_add (images, image);
      fpi_device_enroll_progress (dev, stage + 1, NULL, NULL);

      if (stage + 1 < GDIX51C0_ENROLL_SAMPLES)
        gdix51c0_wait_for_lift_report (self);
    }

  fpi_device_get_enroll_data (dev, &print);
  fpi_print_set_type (print, FPI_PRINT_RAW);

  {
    GVariantBuilder builder;
    g_autoptr(GVariant) data = NULL;

    g_variant_builder_init (&builder, G_VARIANT_TYPE ("aay"));
    for (guint i = 0; i < images->len; i++)
      {
        guint8 *image = g_ptr_array_index (images, i);
        g_variant_builder_add (&builder, "@ay",
                               g_variant_new_fixed_array (G_VARIANT_TYPE_BYTE,
                                                          image,
                                                          GDIX51C0_FRAME_PIXELS,
                                                          sizeof (guint8)));
      }

    data = g_variant_ref_sink (g_variant_builder_end (&builder));
    g_object_set (G_OBJECT (print), "fpi-data", data, NULL);
  }

  self->skip_next_identify = TRUE;
  fp_info ("gdix51c0: enrollment complete with %d SIGFM samples",
           GDIX51C0_ENROLL_SAMPLES);

out:
  gdix51c0_session_deactivate (self);
  if (error)
    fpi_device_enroll_complete (dev, NULL, error);
  else
    fpi_device_enroll_complete (dev, g_object_ref (print), NULL);
}

static void
gdix51c0_verify_or_identify (FpDevice *dev)
{
  FpiDeviceGdix51c0 *self = FPI_DEVICE_GDIX51C0 (dev);
  FpiDeviceAction action = fpi_device_get_current_action (dev);
  g_autofree guint8 *image = NULL;
  SigfmImgInfo *probe_info = NULL;
  GError *error = NULL;

  if (!gdix51c0_session_activate (self, &error))
    goto out;

  fpi_device_report_finger_status (dev, FP_FINGER_STATUS_NEEDED);
  image = gdix51c0_capture_one_raw8 (self, &error);
  if (!image)
    goto out;

  probe_info = sigfm_extract (image, GDIX51C0_SENSOR_WIDTH, GDIX51C0_SENSOR_HEIGHT);
  gint probe_kp = sigfm_keypoints_count (probe_info);

  fp_dbg ("gdix51c0: SIGFM probe keypoints: %d", probe_kp);

  if (probe_kp < GDIX51C0_SIGFM_PROBE_KP_MIN)
    {
      fp_dbg ("gdix51c0: probe quality too low for secure match: keypoints=%d, min=%d",
              probe_kp, GDIX51C0_SIGFM_PROBE_KP_MIN);

      if (action == FPI_DEVICE_ACTION_VERIFY)
        {
          fpi_device_verify_report (dev, FPI_MATCH_FAIL, NULL, NULL);
          goto out;
        }
      else
        {
          fpi_device_identify_report (dev, NULL, NULL, NULL);
          goto out;
        }
    }

  if (action == FPI_DEVICE_ACTION_VERIFY)
    {
      FpPrint *template = NULL;
      gint best_score = 0;
      gint weak_samples = 0;
      gint medium_samples = 0;
      FpiMatchResult result;

      fpi_device_get_verify_data (dev, &template);
      best_score = gdix51c0_match_print (probe_info, template,
                                         &weak_samples, &medium_samples, &error);
      if (error)
        goto out;
      
      result = gdix51c0_sigfm_is_match (best_score, weak_samples, medium_samples, probe_kp) ?
              FPI_MATCH_SUCCESS : FPI_MATCH_FAIL;

      fp_dbg ("gdix51c0: verify decision: best=%d weak_samples=%d medium_samples=%d result=%s",
              best_score,
              weak_samples,
              medium_samples,
              result == FPI_MATCH_SUCCESS ? "MATCH" : "NO_MATCH");

      fpi_device_verify_report (dev, result, NULL, NULL);
    }
  else
    {
      GPtrArray *gallery = NULL;
      FpPrint *match = NULL;
      gint best_score = 0;
      gint best_matching_medium_samples = 0;
      gint best_matching_weak_samples = 0;

      fpi_device_get_identify_data (dev, &gallery);
      for (guint i = 0; i < gallery->len; i++)
        {
          FpPrint *candidate = g_ptr_array_index (gallery, i);
          gint weak_samples = 0;
          gint medium_samples = 0;
          gint score = gdix51c0_match_print (probe_info, candidate,
                                             &weak_samples, &medium_samples, &error);
          if (error)
            goto out;

          if (gdix51c0_sigfm_is_match (score, weak_samples, medium_samples, probe_kp) &&
              score > best_score)
            {
              best_score = score;
              best_matching_medium_samples = medium_samples;
              best_matching_weak_samples = weak_samples;
              match = candidate;
            }
        }

      fp_dbg ("gdix51c0: identify decision: best=%d weak_samples=%d medium_samples=%d result=%s",
        best_score,
        best_matching_weak_samples,
        best_matching_medium_samples,
        match ? "MATCH" : "NO_MATCH");
      fpi_device_identify_report (dev, match, NULL, NULL);
    }

out:
  if (probe_info)
    sigfm_free_info (probe_info);
  if (image)
    gdix51c0_wait_for_lift_report (self);
  gdix51c0_session_deactivate (self);

  if (action == FPI_DEVICE_ACTION_VERIFY)
    fpi_device_verify_complete (dev, error);
  else
    fpi_device_identify_complete (dev, error);
}

static void
gdix51c0_verify (FpDevice *dev)
{
  gdix51c0_verify_or_identify (dev);
}

static void
gdix51c0_identify (FpDevice *dev)
{
  FpiDeviceGdix51c0 *self = FPI_DEVICE_GDIX51C0 (dev);

  if (self->skip_next_identify)
    {
      self->skip_next_identify = FALSE;
      fp_dbg ("gdix51c0: skipping post-enrollment duplicate check");
      fpi_device_identify_report (dev, NULL, NULL, NULL);
      fpi_device_identify_complete (dev, NULL);
      return;
    }

  gdix51c0_verify_or_identify (dev);
}

/* ------------------------------------------------------------------ */
/* GObject boilerplate                                                 */
/* ------------------------------------------------------------------ */

static void
fpi_device_gdix51c0_init (FpiDeviceGdix51c0 *self)
{
  self->spi_fd = -1;
}

static void
fpi_device_gdix51c0_finalize (GObject *gobject)
{
  gdix51c0_session_deactivate (FPI_DEVICE_GDIX51C0 (gobject));
  G_OBJECT_CLASS (fpi_device_gdix51c0_parent_class)->finalize (gobject);
}

static void
fpi_device_gdix51c0_class_init (FpiDeviceGdix51c0Class *klass)
{
  FpDeviceClass *dev_class = FP_DEVICE_CLASS (klass);

  dev_class->id               = "gdix51c0";
  dev_class->full_name        = "Goodix GDIX51C0 Fingerprint Sensor";
  dev_class->type             = FP_DEVICE_TYPE_UDEV;
  dev_class->id_table         = gdix51c0_id_table;
  dev_class->scan_type        = FP_SCAN_TYPE_PRESS;
  /* Bumped from 5 to 10: with a tiny 64x80 (~3-4mm) sensor each press
   * covers a small area, so a single template doesn't span enough finger
   * to reliably overlap with a random-position verify capture.  More
   * stages -> more template coverage -> higher hit rate. */
  dev_class->nr_enroll_stages = GDIX51C0_ENROLL_SAMPLES;
  dev_class->temp_hot_seconds = -1;
  dev_class->open             = gdix51c0_open;
  dev_class->close            = gdix51c0_close;
  dev_class->enroll           = gdix51c0_enroll;
  dev_class->verify           = gdix51c0_verify;
  dev_class->identify         = gdix51c0_identify;

  fpi_device_class_auto_initialize_features (dev_class);

  G_OBJECT_CLASS (klass)->finalize = fpi_device_gdix51c0_finalize;
}
