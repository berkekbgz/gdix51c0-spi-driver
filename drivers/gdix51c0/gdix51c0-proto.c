/*
 * Goodix GDIX51C0 wire protocol — implementation.
 *
 * Synchronous helpers used during open/activate.  We use libfprint's
 * fpi_spi_transfer_submit_sync for the actual SPI traffic so the driver
 * stays well-behaved under cancellation.
 */

#define FP_COMPONENT "gdix51c0"

#include <errno.h>
#include <string.h>

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

guint8 *
gdix51c0_spi_read (FpDevice *dev, int spi_fd, gsize *out_len, GError **error)
{
  /* Mirror python-spidev's readbytes() / xfer2: read() syscall on spidev
   * does a half-duplex read (MOSI idle).  Match python's chunking. */
  (void) dev;
  guint8 hdr[4];
  ssize_t n = read (spi_fd, hdr, 4);
  if (n != 4)
    {
      g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                   "gdix51c0: read header: %zd/%d", n, 4);
      return NULL;
    }
  if (hdr[0] == 0 && hdr[1] == 0 && hdr[2] == 0 && hdr[3] == 0)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                           "gdix51c0: read returned all-zero header");
      return NULL;
    }
  guint16 length = (guint16) hdr[1] | ((guint16) hdr[2] << 8);
  if (length == 0xffff)
    {
      g_set_error_literal (error, G_IO_ERROR, G_IO_ERROR_FAILED,
                           "gdix51c0: read returned all-FF header");
      return NULL;
    }
  if (!gdix51c0_header_checksum_ok (hdr))
    fp_warn ("gdix51c0: header checksum mismatch (%02x %02x %02x %02x)",
             hdr[0], hdr[1], hdr[2], hdr[3]);

  g_autofree guint8 *payload = g_malloc (length);
  gsize remaining = length;
  guint8 *cursor = payload;
  while (remaining > 0)
    {
      gsize chunk = MIN (remaining, (gsize) GDIX51C0_SPI_CHUNK);
      n = read (spi_fd, cursor, chunk);
      if (n <= 0)
        {
          g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                       "gdix51c0: read payload chunk: %zd/%zu", n, chunk);
          return NULL;
        }
      cursor    += n;
      remaining -= n;
    }

  if (out_len)
    *out_len = length;
  return g_steal_pointer (&payload);
}

/* ---------------- IRQ + reset ---------------- */

void
gdix51c0_irq_drain (struct gpiod_line_request *irq_req,
                 struct gpiod_edge_event_buffer *event_buf)
{
  /* Non-blocking poll for any already-buffered edges; discard. */
  while (gpiod_line_request_wait_edge_events (irq_req, 0) > 0)
    gpiod_line_request_read_edge_events (irq_req, event_buf, 16);
}

gboolean
gdix51c0_irq_wait (struct gpiod_line_request *irq_req,
                 struct gpiod_edge_event_buffer *event_buf,
                 gboolean target_high, unsigned int irq_offset,
                 guint timeout_usec, const char *label,
                 GError **error)
{
  enum gpiod_edge_event_type want = target_high
                                    ? GPIOD_EDGE_EVENT_RISING_EDGE
                                    : GPIOD_EDGE_EVENT_FALLING_EDGE;
  gint64 deadline = g_get_monotonic_time () + (gint64) timeout_usec;

  for (;;)
    {
      gint64 now = g_get_monotonic_time ();

      if (now >= deadline)
        break;
      gint64 remaining_ns = (deadline - now) * 1000;

      int ready = gpiod_line_request_wait_edge_events (irq_req, remaining_ns);
      if (ready < 0)
        {
          g_set_error (error, G_IO_ERROR, g_io_error_from_errno (errno),
                       "gdix51c0: %s: wait_edge_events failed", label);
          return FALSE;
        }
      if (ready == 0)
        break; /* timed out */

      /* Consume one edge per wait.  The MCU can emit the next IRQ cycle
       * before the caller arms its next wait (for example FDT ACK fall and
       * finger-detect rise can arrive back-to-back).  Reading a whole batch
       * here would discard those future edges from the kernel queue. */
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
      /* Edges were of the wrong type; loop and wait for more. */
    }

  g_set_error (error, G_IO_ERROR, G_IO_ERROR_TIMED_OUT,
               "gdix51c0: %s: IRQ never went %s",
               label, target_high ? "high" : "low");
  return FALSE;
}

gboolean
gdix51c0_reset_pulse (struct gpiod_chip *chip, unsigned int offset, GError **error)
{
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
gdix51c0_cmd_ack (Gdix51c0Bus *bus, const guint8 *payload, gsize len,
               const char *label, GError **error)
{
  gdix51c0_irq_drain (bus->irq_req, bus->irq_events);
  if (!gdix51c0_spi_write (bus->dev, bus->spi_fd, GDIX51C0_PKT_WRITE, payload, len, error))
    return FALSE;
  if (!gdix51c0_irq_wait (bus->irq_req, bus->irq_events, TRUE,
                       bus->irq_offset, GDIX51C0_CMD_TIMEOUT_USEC, label, error))
    return FALSE;
  if (!gdix51c0_read_and_drop (bus, label, error))
    return FALSE;
  return gdix51c0_irq_wait (bus->irq_req, bus->irq_events, FALSE,
                          bus->irq_offset, GDIX51C0_CMD_TIMEOUT_USEC, label, error);
}

gboolean
gdix51c0_cmd_ack_resp (Gdix51c0Bus *bus, const guint8 *payload, gsize len,
                    const char *label, GError **error)
{
  fp_dbg ("gdix51c0: %s (ack+resp, %zu B)", label, len);
  gdix51c0_irq_drain (bus->irq_req, bus->irq_events);
  if (!gdix51c0_spi_write (bus->dev, bus->spi_fd, GDIX51C0_PKT_WRITE, payload, len, error))
    return FALSE;
  if (!gdix51c0_irq_wait (bus->irq_req, bus->irq_events, TRUE,
                        bus->irq_offset, GDIX51C0_CMD_TIMEOUT_USEC, label, error))
    return FALSE;
  if (!gdix51c0_read_and_drop (bus, label, error))         /* ack */
    return FALSE;
  g_usleep (50000);                                     /* matches manual_sleep(0.05) */
  if (!gdix51c0_read_and_drop (bus, label, error))         /* response */
    return FALSE;
  return gdix51c0_irq_wait (bus->irq_req, bus->irq_events, FALSE,
                          bus->irq_offset, GDIX51C0_CMD_TIMEOUT_USEC, label, error);
}

guint8 *
gdix51c0_cmd_read (Gdix51c0Bus *bus, const guint8 *payload, gsize len,
                guint timeout_usec, gsize *out_len,
                const char *label, GError **error)
{
  fp_dbg ("gdix51c0: %s (read, %zu B)", label, len);
  gdix51c0_irq_drain (bus->irq_req, bus->irq_events);
  if (!gdix51c0_spi_write (bus->dev, bus->spi_fd, GDIX51C0_PKT_WRITE,
                        payload, len, error))
    return NULL;
  if (!gdix51c0_irq_wait (bus->irq_req, bus->irq_events, TRUE,
                        bus->irq_offset, timeout_usec, label, error))
    return NULL;
  guint8 *buf = gdix51c0_spi_read (bus->dev, bus->spi_fd, out_len, error);
  if (!buf)
    return NULL;
  if (!gdix51c0_irq_wait (bus->irq_req, bus->irq_events, FALSE,
                        bus->irq_offset, timeout_usec, label, error))
    {
      g_free (buf);
      return NULL;
    }
  fp_dbg ("gdix51c0: %s read %zu B", label, out_len ? *out_len : 0);
  return buf;
}

gboolean
gdix51c0_cmd_ack_then_resp (Gdix51c0Bus *bus, const guint8 *payload, gsize len,
                         const char *label, GError **error)
{
  gdix51c0_irq_drain (bus->irq_req, bus->irq_events);
  if (!gdix51c0_spi_write (bus->dev, bus->spi_fd, GDIX51C0_PKT_WRITE, payload, len, error))
    return FALSE;
  /* first IRQ cycle: ack */
  if (!gdix51c0_irq_wait (bus->irq_req, bus->irq_events, TRUE,
                        bus->irq_offset, GDIX51C0_CMD_TIMEOUT_USEC, label, error))
    return FALSE;
  if (!gdix51c0_read_and_drop (bus, label, error))
    return FALSE;
  if (!gdix51c0_irq_wait (bus->irq_req, bus->irq_events, FALSE,
                        bus->irq_offset, GDIX51C0_CMD_TIMEOUT_USEC, label, error))
    return FALSE;
  /* second IRQ cycle: response */
  if (!gdix51c0_irq_wait (bus->irq_req, bus->irq_events, TRUE,
                        bus->irq_offset, GDIX51C0_CMD_TIMEOUT_USEC, label, error))
    return FALSE;
  if (!gdix51c0_read_and_drop (bus, label, error))
    return FALSE;
  return gdix51c0_irq_wait (bus->irq_req, bus->irq_events, FALSE,
                          bus->irq_offset, GDIX51C0_CMD_TIMEOUT_USEC, label, error);
}
