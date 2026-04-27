/*
 * Goodix GDIX51C0 wire protocol — implementation.
 *
 * Synchronous helpers used during open/activate.  We use libfprint's
 * fpi_spi_transfer_submit_sync for the actual SPI traffic so the driver
 * stays well-behaved under cancellation.
 */

#define FP_COMPONENT "gdix51c0"

#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <linux/spi/spidev.h>

#include "drivers_api.h"
#include "fpi-spi-transfer.h"

#include "gdix51c0.h"
#include "gdix51c0-proto.h"

/* ---------------- packet construction ---------------- */

guint8
gdix51c0_payload_checksum (const guint8 *buf, gsize len)
{
  guint sum = 0;
  for (gsize i = 0; i < len; i++)
    sum += buf[i];
  return (guint8) ((0xaa - sum) & 0xff);
}

gboolean
gdix51c0_header_checksum_ok (const guint8 *hdr4)
{
  guint sum = (guint) hdr4[0] + hdr4[1] + hdr4[2];
  return ((sum & 0xff) == hdr4[3]);
}

guint8 *
gdix51c0_make_header_packet (guint8 type, guint16 payload_len)
{
  guint8 *pkt = g_malloc (4);
  pkt[0] = type;
  pkt[1] = payload_len & 0xff;
  pkt[2] = (payload_len >> 8) & 0xff;
  pkt[3] = (guint8) ((pkt[0] + pkt[1] + pkt[2]) & 0xff);
  return pkt;
}

guint8 *
gdix51c0_make_payload_packet (guint8 type, const guint8 *data, gsize data_len,
                           gsize *out_len)
{
  /* type | LE16 inner_len | data | csum   where inner_len = data_len + 1 */
  guint16 inner_len = (guint16) (data_len + 1);
  gsize total = 1 + 2 + data_len + 1;
  guint8 *pkt = g_malloc (total);

  pkt[0] = type;
  pkt[1] = inner_len & 0xff;
  pkt[2] = (inner_len >> 8) & 0xff;
  if (data_len > 0)
    memcpy (&pkt[3], data, data_len);
  pkt[total - 1] = gdix51c0_payload_checksum (pkt, total - 1);

  if (out_len)
    *out_len = total;
  return pkt;
}

/* ---------------- SPI sync I/O ---------------- */

gboolean
gdix51c0_spi_write (FpDevice *dev, int spi_fd,
                 guint8 outer_type,
                 const guint8 *payload, gsize payload_len,
                 GError **error)
{
  /* Mirror python-spidev's writebytes() exactly: one ioctl per buffer.
   * Header and payload as two SPI transactions (two CS cycles).        */
  (void) dev;
  g_autofree guint8 *hdr = gdix51c0_make_header_packet (outer_type, (guint16) payload_len);
  ssize_t n = write (spi_fd, hdr, 4);
  if (n != 4)
    {
      g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: write header: %zd/%d", n, 4);
      return FALSE;
    }
  if (payload_len == 0)
    return TRUE;
  n = write (spi_fd, payload, payload_len);
  if ((gsize) n != payload_len)
    {
      g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: write payload: %zd/%zu", n, payload_len);
      return FALSE;
    }
  return TRUE;
}

#define GDIX51C0_SPI_READ_TAIL_GUARD 2

static gboolean
gdix51c0_spi_xfer_read (int spi_fd,
                        guint8 *rx,
                        gsize len,
                        GError **error)
{
  gsize done = 0;

  while (done < len)
    {
      gsize chunk = MIN (len - done, (gsize) 2048);
      g_autofree guint8 *tx = g_malloc (chunk);

      memset (tx, 0xff, chunk);

      struct spi_ioc_transfer tr = {
        .tx_buf = (uintptr_t) tx,
        .rx_buf = (uintptr_t) (rx + done),
        .len = chunk,
        .delay_usecs = 0,
        .speed_hz = 0,       /* use fd-configured speed */
        .bits_per_word = 8,
      };

      int ret = ioctl (spi_fd, SPI_IOC_MESSAGE (1), &tr);
      if (ret < 1)
        {
          g_set_error (error,
                       G_IO_ERROR,
                       g_io_error_from_errno (errno),
                       "gdix51c0: SPI_IOC_MESSAGE read failed at %zu/%zu",
                       done,
                       len);
          return FALSE;
        }

      done += chunk;
    }

  return TRUE;
}

guint8 *
gdix51c0_spi_read (FpDevice *dev, int spi_fd, gsize *out_len, GError **error)
{
  /*
   * Match protocol_interaction.py:
   *
   *   spi.xfer2([0xff] * n)
   *
   * Do not use plain read(). On this platform, half-duplex read timing can
   * produce corrupted/truncated payload bytes, which makes decrypted image
   * frames look like noise even when the command sequence is correct.
   */
  (void) dev;

  guint8 hdr[4];
  gboolean saw_all_zero = FALSE;
  gboolean saw_all_ff = FALSE;

  for (guint attempt = 0; attempt < 4; attempt++)
    {
      if (!gdix51c0_spi_xfer_read (spi_fd, hdr, sizeof (hdr), error))
        return NULL;

      saw_all_zero = hdr[0] == 0 && hdr[1] == 0 && hdr[2] == 0 && hdr[3] == 0;
      saw_all_ff = hdr[0] == 0xff && hdr[1] == 0xff && hdr[2] == 0xff && hdr[3] == 0xff;

      if (!saw_all_zero && !saw_all_ff)
        break;

      if (attempt + 1 == 4)
        break;

      fp_dbg ("gdix51c0: SPI read saw idle %s header, retrying",
              saw_all_ff ? "all-FF" : "all-zero");
      g_usleep (5000 + attempt * 10000);
    }

  if (saw_all_zero)
    {
      g_set_error_literal (error,
                           G_IO_ERROR,
                           G_IO_ERROR_FAILED,
                           "gdix51c0: read returned all-zero header");
      return NULL;
    }

  guint16 length = (guint16) hdr[1] | ((guint16) hdr[2] << 8);

  if (saw_all_ff || length == 0xffff)
    {
      g_set_error_literal (error,
                           G_IO_ERROR,
                           G_IO_ERROR_FAILED,
                           "gdix51c0: read returned all-FF header");
      return NULL;
    }

  if (!gdix51c0_header_checksum_ok (hdr))
    {
      fp_warn ("gdix51c0: header checksum mismatch (%02x %02x %02x %02x)",
               hdr[0], hdr[1], hdr[2], hdr[3]);
    }

  gsize read_len = (gsize) length + GDIX51C0_SPI_READ_TAIL_GUARD;
  g_autofree guint8 *payload_with_guard = g_malloc (read_len);

  if (read_len > 0 &&
      !gdix51c0_spi_xfer_read (spi_fd, payload_with_guard, read_len, error))
    return NULL;

  if (GDIX51C0_SPI_READ_TAIL_GUARD > 0 && read_len > length)
    {
      const guint8 *tail = payload_with_guard + length;
      gboolean nonzero_tail = FALSE;

      for (gsize i = 0; i < GDIX51C0_SPI_READ_TAIL_GUARD; i++)
        {
          if (tail[i] != 0x00)
            {
              nonzero_tail = TRUE;
              break;
            }
        }

      if (nonzero_tail && !(tail[0] == 0xff && tail[1] == 0xff))
        {
          fp_dbg ("gdix51c0: SPI read tail guard after payload was %02x %02x",
                  tail[0],
                  GDIX51C0_SPI_READ_TAIL_GUARD > 1 ? tail[1] : 0);
        }
    }

  guint8 *payload = g_malloc (length);
  if (length > 0)
    memcpy (payload, payload_with_guard, length);

  if (out_len)
    *out_len = length;

  return payload;
}

/* ---------------- IRQ + reset ---------------- */

void
gdix51c0_irq_drain (struct gpiod_line_request *irq_req,
                 struct gpiod_edge_event_buffer *event_buf)
{
  if (!irq_req || !event_buf)
    return;
  /* Non-blocking poll for any already-buffered edges; discard. */
  while (gpiod_line_request_wait_edge_events (irq_req, 0) > 0)
    gpiod_line_request_read_edge_events (irq_req, event_buf, 16);
}

gboolean
gdix51c0_irq_wait (struct gpiod_line_request *irq_req,
                   struct gpiod_edge_event_buffer *event_buf,
                   gboolean target_high,
                   unsigned int irq_offset,
                   guint timeout_usec,
                   const char *label,
                   GError **error)
{
  enum gpiod_line_value value;
  enum gpiod_edge_event_type want = target_high
                                    ? GPIOD_EDGE_EVENT_RISING_EDGE
                                    : GPIOD_EDGE_EVENT_FALLING_EDGE;
  gint64 deadline = g_get_monotonic_time () + (gint64) timeout_usec;

  if (!irq_req || !event_buf)
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_CLOSED,
                    "gdix51c0: %s: IRQ line is closed", label);
      return FALSE;
    }

  /* Critical fix: if the line is already at the desired level,
   * do not wait for an edge that has already happened. */
  value = gpiod_line_request_get_value (irq_req, irq_offset);
  if (value < 0)
    {
      g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: %s: get IRQ value failed", label);
      return FALSE;
    }

  if ((value == GPIOD_LINE_VALUE_ACTIVE) == target_high)
    return TRUE;

  for (;;)
    {
      gint64 now = g_get_monotonic_time ();

      if (now >= deadline)
        break;

      int ready = gpiod_line_request_wait_edge_events (
        irq_req, (deadline - now) * 1000);

      if (ready < 0)
        {
          g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                       "gdix51c0: %s: wait_edge_events failed", label);
          return FALSE;
        }

      if (ready == 0)
        break;

      int n = gpiod_line_request_read_edge_events (irq_req, event_buf, 1);
      if (n < 0)
        {
          g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                       "gdix51c0: %s: read_edge_events failed", label);
          return FALSE;
        }

      for (int i = 0; i < n; i++)
        {
          struct gpiod_edge_event *ev =
            gpiod_edge_event_buffer_get_event (event_buf, i);

          if (ev && gpiod_edge_event_get_event_type (ev) == want)
            return TRUE;
        }

      /* Also re-check physical level after any edge. This handles cases
       * where gpiod delivered the opposite edge from an old transition,
       * but the line is now already in the target state. */
      value = gpiod_line_request_get_value (irq_req, irq_offset);
      if (value < 0)
        {
          g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                       "gdix51c0: %s: get IRQ value failed", label);
          return FALSE;
        }

      if ((value == GPIOD_LINE_VALUE_ACTIVE) == target_high)
        return TRUE;
    }

  value = gpiod_line_request_get_value (irq_req, irq_offset);
  if (value >= 0)
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                   "gdix51c0: %s: IRQ never went %s; final level=%s",
                   label,
                   target_high ? "high" : "low",
                   value == GPIOD_LINE_VALUE_ACTIVE ? "high" : "low");
    }
  else
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                   "gdix51c0: %s: IRQ never went %s",
                   label,
                   target_high ? "high" : "low");
    }

  return FALSE;
}

gboolean
gdix51c0_reset_pulse (struct gpiod_chip *chip, unsigned int offset, GError **error)
{
  if (!chip)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_CLOSED,
                          "gdix51c0: reset requested but gpio chip is closed");
      return FALSE;
    }
  /* Mirror python: open the line as OUTPUT(0), write 1, sleep, write 0,
   * sleep, then RELEASE the request entirely.  The release lets the
   * kernel return the line to its default state (INPUT/high-Z), and
   * the platform pull-up brings the MCU out of reset.  Reconfiguring
   * an existing OUTPUT(0) request to INPUT is NOT equivalent — the
   * kernel keeps remembering the OUTPUT-low intent on this platform. */
  struct gpiod_line_settings  *settings = gpiod_line_settings_new ();
  struct gpiod_line_config    *cfg      = gpiod_line_config_new ();
  struct gpiod_request_config *req_cfg  = gpiod_request_config_new ();
  struct gpiod_line_request   *req      = NULL;
  gboolean ok = FALSE;

  if (!settings || !cfg || !req_cfg)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                           "gdix51c0: gpiod alloc failed");
      goto out;
    }

  gpiod_line_settings_set_direction    (settings, GPIOD_LINE_DIRECTION_OUTPUT);
  gpiod_line_settings_set_output_value (settings, GPIOD_LINE_VALUE_INACTIVE);
  if (gpiod_line_config_add_line_settings (cfg, &offset, 1, settings) < 0)
    {
      g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: reset add line %u failed", offset);
      goto out;
    }
  gpiod_request_config_set_consumer (req_cfg, "gdix51c0-reset");

  req = gpiod_chip_request_lines (chip, req_cfg, cfg);
  if (!req)
    {
      g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: reset request line %u failed", offset);
      goto out;
    }

  gpiod_line_request_set_value (req, offset, GPIOD_LINE_VALUE_ACTIVE);
  g_usleep (10000);
  gpiod_line_request_set_value (req, offset, GPIOD_LINE_VALUE_INACTIVE);
  g_usleep (10000);

  ok = TRUE;
out:
  if (req)      gpiod_line_request_release (req);
  if (settings) gpiod_line_settings_free (settings);
  if (cfg)      gpiod_line_config_free (cfg);
  if (req_cfg)  gpiod_request_config_free (req_cfg);
  return ok;
}

guint8
gdix51c0_payload_checksum_ts (const guint8 *buf, gsize len)
{
  return (guint8) (gdix51c0_payload_checksum (buf, len) + 1);
}

/* ---------------- 14213 init-sequence helpers ---------------- */

#define GDIX51C0_CMD_TIMEOUT_USEC (5 * 1000 * 1000)

gboolean
gdix51c0_cmd_no_ack (Gdix51c0Bus *bus, const guint8 *payload, gsize len,
                  const char *label, GError **error)
{
  fp_dbg ("gdix51c0: %s (no-ack, %zu B)", label, len);
  return gdix51c0_spi_write (bus->dev, bus->spi_fd, GDIX51C0_PKT_WRITE,
                          payload, len, error);
}

static gboolean
gdix51c0_read_and_drop (Gdix51c0Bus *bus, const char *label, GError **error)
{
  gsize n = 0;
  g_autofree guint8 *buf = gdix51c0_spi_read (bus->dev, bus->spi_fd, &n, error);
  if (!buf)
    return FALSE;
  fp_dbg ("gdix51c0: %s read %zu B", label, n);
  return TRUE;
}

gboolean
gdix51c0_cmd_ack (Gdix51c0Bus *bus,
                  const guint8 *payload,
                  gsize len,
                  const char *label,
                  GError **error)
{
  g_usleep (20000);
  gdix51c0_irq_drain (bus->irq_req, bus->irq_events);

  if (!gdix51c0_spi_write (bus->dev, bus->spi_fd,
                           GDIX51C0_PKT_WRITE, payload, len, error))
    return FALSE;

  if (!gdix51c0_irq_wait_edge_strict (bus->irq_req, bus->irq_events, TRUE,
                                      bus->irq_offset,
                                      GDIX51C0_CMD_TIMEOUT_USEC,
                                      label, error))
    return FALSE;

  if (!gdix51c0_read_and_drop (bus, label, error))
    return FALSE;

  return gdix51c0_irq_wait_edge_strict (bus->irq_req, bus->irq_events, FALSE,
                                        bus->irq_offset,
                                        GDIX51C0_CMD_TIMEOUT_USEC,
                                        label, error);
}

gboolean
gdix51c0_cmd_ack_resp (Gdix51c0Bus *bus, const guint8 *payload, gsize len,
                    const char *label, GError **error)
{
  fp_dbg ("gdix51c0: %s (ack then resp, %zu B)", label, len);
  return gdix51c0_cmd_ack_then_resp (bus, payload, len, label, error);
}

guint8 *
gdix51c0_cmd_read (Gdix51c0Bus *bus,
                   const guint8 *payload,
                   gsize len,
                   guint timeout_usec,
                   gsize *out_len,
                   const char *label,
                   GError **error)
{
  fp_dbg ("gdix51c0: %s (read, %zu B)", label, len);

  gdix51c0_irq_drain (bus->irq_req, bus->irq_events);

  if (!gdix51c0_spi_write (bus->dev, bus->spi_fd,
                           GDIX51C0_PKT_WRITE, payload, len, error))
    return NULL;

  if (!gdix51c0_irq_wait_edge_strict (bus->irq_req, bus->irq_events, TRUE,
                                      bus->irq_offset, timeout_usec,
                                      label, error))
    return NULL;

  guint8 *buf = gdix51c0_spi_read (bus->dev, bus->spi_fd, out_len, error);
  if (!buf)
    return NULL;

  if (!gdix51c0_irq_wait_edge_strict (bus->irq_req, bus->irq_events, FALSE,
                                      bus->irq_offset, timeout_usec,
                                      label, error))
    {
      g_free (buf);
      return NULL;
    }

  fp_dbg ("gdix51c0: %s read %zu B", label, out_len ? *out_len : 0);
  return buf;
}

gboolean
gdix51c0_cmd_ack_then_resp (Gdix51c0Bus *bus,
                            const guint8 *payload,
                            gsize len,
                            const char *label,
                            GError **error)
{
  g_usleep (20000);
  gdix51c0_irq_drain (bus->irq_req, bus->irq_events);

  if (!gdix51c0_spi_write (bus->dev, bus->spi_fd,
                           GDIX51C0_PKT_WRITE, payload, len, error))
    return FALSE;

  if (!gdix51c0_irq_wait_edge_strict (bus->irq_req, bus->irq_events, TRUE,
                                      bus->irq_offset,
                                      GDIX51C0_CMD_TIMEOUT_USEC,
                                      label, error))
    return FALSE;

  if (!gdix51c0_read_and_drop (bus, label, error))
    return FALSE;

  if (!gdix51c0_irq_wait_edge_strict (bus->irq_req, bus->irq_events, FALSE,
                                      bus->irq_offset,
                                      GDIX51C0_CMD_TIMEOUT_USEC,
                                      label, error))
    return FALSE;

  if (!gdix51c0_irq_wait_edge_strict (bus->irq_req, bus->irq_events, TRUE,
                                      bus->irq_offset,
                                      GDIX51C0_CMD_TIMEOUT_USEC,
                                      label, error))
    return FALSE;

  if (!gdix51c0_read_and_drop (bus, label, error))
    return FALSE;

  return gdix51c0_irq_wait_edge_strict (bus->irq_req, bus->irq_events, FALSE,
                                        bus->irq_offset,
                                        GDIX51C0_CMD_TIMEOUT_USEC,
                                        label, error);
}

guint8 *
gdix51c0_cmd_single_resp (Gdix51c0Bus *bus,
                          const guint8 *payload,
                          gsize len,
                          guint timeout_usec,
                          gsize *out_len,
                          const char *label,
                          GError **error)
{
  fp_dbg ("gdix51c0: %s (single response, %zu B)", label, len);

  g_usleep (20000);
  gdix51c0_irq_drain (bus->irq_req, bus->irq_events);

  if (!gdix51c0_spi_write (bus->dev, bus->spi_fd,
                           GDIX51C0_PKT_WRITE, payload, len, error))
    return NULL;

  if (!gdix51c0_irq_wait_edge_strict (bus->irq_req, bus->irq_events, TRUE,
                                      bus->irq_offset,
                                      timeout_usec,
                                      label, error))
    return NULL;

  guint8 *buf = gdix51c0_spi_read (bus->dev, bus->spi_fd, out_len, error);
  if (!buf)
    return NULL;

  if (!gdix51c0_irq_wait_edge_strict (bus->irq_req, bus->irq_events, FALSE,
                                      bus->irq_offset,
                                      timeout_usec,
                                      label, error))
    {
      g_free (buf);
      return NULL;
    }

  fp_dbg ("gdix51c0: %s single response read %zu B",
          label, out_len ? *out_len : 0);

  return buf;
}

gboolean
gdix51c0_cmd_ack_optional_resp (Gdix51c0Bus *bus,
                                const guint8 *payload,
                                gsize len,
                                guint optional_timeout_usec,
                                const char *label,
                                GError **error)
{
  g_usleep (20000);
  gdix51c0_irq_drain (bus->irq_req, bus->irq_events);

  fp_dbg ("gdix51c0: %s (ack + optional resp, %zu B)", label, len);

  if (!gdix51c0_spi_write (bus->dev,
                           bus->spi_fd,
                           GDIX51C0_PKT_WRITE,
                           payload,
                           len,
                           error))
    return FALSE;

  /* Required ACK cycle. */
  if (!gdix51c0_irq_wait (bus->irq_req,
                          bus->irq_events,
                          TRUE,
                          bus->irq_offset,
                          GDIX51C0_CMD_TIMEOUT_USEC,
                          label,
                          error))
    return FALSE;

  if (!gdix51c0_read_and_drop (bus, label, error))
    return FALSE;

  if (!gdix51c0_irq_wait (bus->irq_req,
                          bus->irq_events,
                          FALSE,
                          bus->irq_offset,
                          GDIX51C0_CMD_TIMEOUT_USEC,
                          label,
                          error))
    return FALSE;

  /*
   * Some init commands sometimes produce a second 6-byte status/ACK-like
   * packet, and sometimes do not. Do not fail if it is absent.
   */
  if (gpiod_line_request_wait_edge_events (bus->irq_req,
                                           optional_timeout_usec * 1000) > 0)
    {
      if (!gdix51c0_irq_wait (bus->irq_req,
                              bus->irq_events,
                              TRUE,
                              bus->irq_offset,
                              optional_timeout_usec,
                              label,
                              NULL))
        return TRUE;

      g_autoptr(GError) local_error = NULL;
      if (!gdix51c0_read_and_drop (bus, label, &local_error))
        {
          fp_dbg ("gdix51c0: %s optional response read failed: %s",
                   label,
                   local_error ? local_error->message : "?");
          return TRUE;
        }

      gdix51c0_irq_wait (bus->irq_req,
                         bus->irq_events,
                         FALSE,
                         bus->irq_offset,
                         optional_timeout_usec,
                         label,
                         NULL);
    }

  return TRUE;
}

gboolean
gdix51c0_irq_wait_edge_strict (struct gpiod_line_request *irq_req,
                               struct gpiod_edge_event_buffer *event_buf,
                               gboolean target_high,
                               unsigned int irq_offset,
                               guint timeout_usec,
                               const char *label,
                               GError **error)
{
  enum gpiod_edge_event_type want =
    target_high ? GPIOD_EDGE_EVENT_RISING_EDGE : GPIOD_EDGE_EVENT_FALLING_EDGE;

  gint64 deadline = g_get_monotonic_time () + (gint64) timeout_usec;

  if (!irq_req || !event_buf)
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_CLOSED,
                    "gdix51c0: %s: IRQ line is closed", label);
      return FALSE;
    }

  for (;;)
    {
      gint64 now = g_get_monotonic_time ();
      if (now >= deadline)
        break;

      int ready =
        gpiod_line_request_wait_edge_events (irq_req, (deadline - now) * 1000);

      if (ready < 0)
        {
          g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                       "gdix51c0: %s: wait_edge_events failed", label);
          return FALSE;
        }

      if (ready == 0)
        break;

      int n = gpiod_line_request_read_edge_events (irq_req, event_buf, 1);
      if (n < 0)
        {
          g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                       "gdix51c0: %s: read_edge_events failed", label);
          return FALSE;
        }

      for (int i = 0; i < n; i++)
        {
          struct gpiod_edge_event *ev =
            gpiod_edge_event_buffer_get_event (event_buf, i);

          if (!ev)
            continue;

          if (gpiod_edge_event_get_event_type (ev) != want)
            continue;

          enum gpiod_line_value value =
            gpiod_line_request_get_value (irq_req, irq_offset);

          if (value < 0)
            {
              g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                           "gdix51c0: %s: get IRQ value failed", label);
              return FALSE;
            }

          if ((value == GPIOD_LINE_VALUE_ACTIVE) == target_high)
            return TRUE;
        }
    }

  enum gpiod_line_value value =
    gpiod_line_request_get_value (irq_req, irq_offset);

  if (value >= 0)
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                   "gdix51c0: %s: IRQ edge never went %s; final level=%s",
                   label,
                   target_high ? "high" : "low",
                   value == GPIOD_LINE_VALUE_ACTIVE ? "high" : "low");
    }
  else
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
                   "gdix51c0: %s: IRQ edge never went %s",
                   label,
                   target_high ? "high" : "low");
    }

  return FALSE;
}

guint8 *
gdix51c0_cmd_single_resp_level (Gdix51c0Bus *bus,
                                const guint8 *payload,
                                gsize len,
                                guint timeout_usec,
                                gsize *out_len,
                                const char *label,
                                GError **error)
{
  fp_dbg ("gdix51c0: %s (single response level-wait, %zu B)", label, len);

  g_usleep (20000);
  gdix51c0_irq_drain (bus->irq_req, bus->irq_events);

  if (!gdix51c0_spi_write (bus->dev, bus->spi_fd,
                           GDIX51C0_PKT_WRITE, payload, len, error))
    return NULL;

  if (!gdix51c0_irq_wait (bus->irq_req, bus->irq_events, TRUE,
                          bus->irq_offset, timeout_usec,
                          label, error))
    return NULL;

  guint8 *buf = gdix51c0_spi_read (bus->dev, bus->spi_fd, out_len, error);
  if (!buf)
    return NULL;

  if (!gdix51c0_irq_wait (bus->irq_req, bus->irq_events, FALSE,
                          bus->irq_offset, timeout_usec,
                          label, error))
    {
      g_free (buf);
      return NULL;
    }

  fp_dbg ("gdix51c0: %s single response read %zu B",
          label, out_len ? *out_len : 0);

  return buf;
}
