/*
 * Goodix GDIX51C0 wire protocol — header. Mirrors protocol_interaction.py.
 */

#pragma once

#include <glib.h>
#include <gpiod.h>

#include "fp-device.h"

G_BEGIN_DECLS

/* ---- Packet construction ----------------------------------------- */
/*
 * Wire format (matches python make_header_packet / make_payload_packet):
 *   [header 4B]  type | LE16 length | header_csum=sum(type|length)&0xff
 *   [payload NB] type | LE16 inner_len | data... | payload_csum=(0xaa - sum_so_far) & 0xff
 *
 * Header `type` is the outer wrapper (0xa0=write, 0xb0=read-ack).
 * Payload `type` is the inner cmd byte (0x32 FDT-down, 0x22 image-T1, ...).
 * `inner_len` is len(data) + 1 to include the trailing csum byte.
 */
guint8 *gdix51c0_make_header_packet  (guint8 type, guint16 payload_len);
guint8 *gdix51c0_make_payload_packet (guint8 type, const guint8 *data, gsize data_len, gsize *out_len);

guint8  gdix51c0_payload_checksum    (const guint8 *buf, gsize len);
gboolean gdix51c0_header_checksum_ok (const guint8 *hdr4);

/* ---- Synchronous SPI I/O ----------------------------------------- */
/*
 * write a header (type=0xa0) + payload
 */
gboolean gdix51c0_spi_write (FpDevice *dev, int spi_fd,
                          guint8 outer_type,
                          const guint8 *payload, gsize payload_len,
                          GError **error);

/*
 * read a 4-byte header, then payload_length bytes. Returns the payload
 * (caller frees with g_free).  out_len is set to the payload length.
 */
guint8 *gdix51c0_spi_read (FpDevice *dev, int spi_fd,
                        gsize *out_len, GError **error);

/* ---- IRQ + reset ------------------------------------------------- */
/*
 * Wait for an edge event of the given direction on the IRQ line.
 * target_high=TRUE means "wait for next rising edge", FALSE means
 * "wait for next falling".  Callers that start a new command drain any stale
 * queued edges before writing, but do not drain between ACK and response
 * cycles because those edges can arrive back-to-back.
 */
gboolean gdix51c0_irq_wait (struct gpiod_line_request *irq_req,
                          struct gpiod_edge_event_buffer *event_buf,
                          gboolean target_high, unsigned int irq_offset,
                          guint timeout_usec, const char *label,
                          GError **error);

/*
 * Drain any currently-buffered edge events without blocking.  Use this
 * after the boot probe to discard the post-reset pulse before the cmd
 * pre-roll starts counting edges.
 */
void gdix51c0_irq_drain (struct gpiod_line_request *irq_req,
                      struct gpiod_edge_event_buffer *event_buf);

/*
 * Pulse the reset line high then low.  Opens a fresh request, drives
 * the pulse, then releases the request — exactly like Python's
 * CdevGPIO(...).write(1).write(0).close() sequence.  The release lets
 * the line go to its default state (INPUT/high-Z) so any external
 * pull-up can deassert reset.
 */
gboolean gdix51c0_reset_pulse (struct gpiod_chip *chip, unsigned int offset, GError **error);

/*
 * Variant of payload checksum used for the get-mcu-state cmd 0xaf:
 * standard checksum + 1.  See Python calculate_checksum_for_mcu_timestamp.
 */
guint8 gdix51c0_payload_checksum_ts (const guint8 *buf, gsize len);

/* ---- Convenience wrappers for the 14213 init sequence ---------- */

/*
 * Common context bundle so the init helpers don't need 6 params each.
 */
typedef struct
{
  FpDevice                       *dev;
  int                             spi_fd;
  struct gpiod_line_request      *irq_req;
  unsigned int                    irq_offset;
  struct gpiod_edge_event_buffer *irq_events;
} Gdix51c0Bus;

/*
 * write a payload (outer 0xa0). No IRQ wait, no read.
 */
gboolean gdix51c0_cmd_no_ack (Gdix51c0Bus *bus, const guint8 *payload, gsize len,
                           const char *label, GError **error);

/*
 * write payload, wait IRQ rise, read ack, wait IRQ fall. Discards ack.
 */
gboolean gdix51c0_cmd_ack (Gdix51c0Bus *bus, const guint8 *payload, gsize len,
                        const char *label, GError **error);

/*
 * write payload, wait IRQ rise, read ack, sleep 50ms, read response,
 * wait IRQ fall. Both buffers freed.
 */
gboolean gdix51c0_cmd_ack_resp (Gdix51c0Bus *bus, const guint8 *payload, gsize len,
                             const char *label, GError **error);

/*
 * write payload, wait IRQ rise, read ack, wait IRQ fall, wait next
 * IRQ rise, read response, wait IRQ fall. Used by cmd 0xa6 (OTP) and
 * reset_sensor (cmd 0xa2).
 */
gboolean gdix51c0_cmd_ack_then_resp (Gdix51c0Bus *bus, const guint8 *payload, gsize len,
                                  const char *label, GError **error);

/*
 * write payload, wait IRQ rise, read ack, wait IRQ fall.  Returns the
 * just-read response buffer (caller g_free), and out_len is set to the
 * bytes read.  Same shape as cmd_ack but exposes the response data.
 */
guint8 *gdix51c0_cmd_read (Gdix51c0Bus *bus, const guint8 *payload, gsize len,
                        guint timeout_usec, gsize *out_len,
                        const char *label, GError **error);

guint8 *
gdix51c0_cmd_single_resp (Gdix51c0Bus *bus,
                          const guint8 *payload,
                          gsize len,
                          guint timeout_usec,
                          gsize *out_len,
                          const char *label,
                          GError **error);

G_END_DECLS

gboolean
gdix51c0_cmd_ack_optional_resp (Gdix51c0Bus *bus,
                                const guint8 *payload,
                                gsize len,
                                guint optional_timeout_usec,
                                const char *label,
                                GError **error);


gboolean
gdix51c0_irq_wait_edge_strict (struct gpiod_line_request *irq_req,
                               struct gpiod_edge_event_buffer *event_buf,
                               gboolean target_high,
                               unsigned int irq_offset,
                               guint timeout_usec,
                               const char *label,
                               GError **error);
