/*
 * Goodix GDIX51C0 TLS layer (OpenSSL 3, server-side PSK).
 *
 * The MCU is the TLS client and we're the server — Goodix did the
 * handshake the wrong way around.  Cipher is PSK-AES128-GCM-SHA256
 * (RFC 5487) over TLSv1.2 only.  Identity hint is "Client_identity".
 */

#pragma once

#include <glib.h>
#include <openssl/ssl.h>

#include "fp-device.h"
#include "gdix51c0-proto.h"

G_BEGIN_DECLS

typedef struct
{
  SSL_CTX *ctx;
  SSL     *ssl;
  BIO     *bio;        /* attached to ssl, owned by ssl */
  /* read-side buffer for our custom BIO: spi packets get pulled in here
   * and OpenSSL drains via BIO_read */
  guint8  *rd_buf;
  gsize    rd_off;
  gsize    rd_len;
  /* dependencies for the BIO callbacks */
  FpDevice                       *dev;
  int                             spi_fd;
  struct gpiod_line_request      *irq_req;
  struct gpiod_edge_event_buffer *irq_events;
  unsigned int                    irq_offset;
  /* PSK; owned by us */
  guint8                     psk[32];
  gsize                      psk_len;
} Gdix51c0Tls;

/*
 * Set up SSL_CTX/SSL with our PSK callback and a custom BIO that talks
 * to the MCU through gdix51c0_proto's SPI helpers.  Caller owns *out_tls
 * and must g_free + free internals via gdix51c0_tls_free.
 */
gboolean gdix51c0_tls_init (Gdix51c0Tls   *tls,
                         FpDevice   *dev,
                         int         spi_fd,
                         struct gpiod_line_request      *irq_req,
                         struct gpiod_edge_event_buffer *irq_events,
                         unsigned int                    irq_offset,
                         const guint8 *psk, gsize psk_len,
                         GError    **error);

/* Drive the handshake to completion.  Sends cmd 0xd1 to kick the MCU,
 * then loops SSL_accept until done.  Times out per GDIX51C0_BOOT_TIMEOUT. */
gboolean gdix51c0_tls_handshake (Gdix51c0Tls *tls, GError **error);

/* Decrypt a TLS application record (e.g. the cmd 0x22 image frame).
 * Returns the plaintext (caller g_free) or NULL on error. */
guint8 *gdix51c0_tls_decrypt_record (Gdix51c0Tls *tls,
                                  const guint8 *record, gsize record_len,
                                  gsize *out_len, GError **error);

gboolean
gdix51c0_tls_record_header_ok (const guint8 *record, gsize record_len);

void     gdix51c0_tls_free (Gdix51c0Tls *tls);

G_END_DECLS
