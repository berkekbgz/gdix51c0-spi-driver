/*
 * Goodix GDIX51C0 TLS layer.
 *
 * Custom BIO that reads/writes whole SPI packets to/from the MCU.
 * On the read side we apply the same workarounds as the Python script:
 *   - The MCU sends a malformed ChangeCipherSpec record with length 0;
 *     patch to length 1, payload 0x01 (per RFC 5246).
 *   - The MCU's ClientKeyExchange identity field has historically come
 *     back with the trailing bytes zeroed (spidev tail-dropout); we
 *     restore "Client_identity" defensively even though the host-level
 *     DMA fix should make this unnecessary on 14213.
 */

#define FP_COMPONENT "gdix51c0"

#include <errno.h>
#include <string.h>

#include <openssl/bio.h>
#include <openssl/err.h>
#include <openssl/ssl.h>

#include "drivers_api.h"

#include "gdix51c0.h"
#include "gdix51c0-proto.h"
#include "gdix51c0-tls.h"

#define GDIX51C0_PSK_IDENTITY "Client_identity"

/* ---------------- Custom BIO ---------------- */

#define GDIX51C0_BIO_NAME "gdix51c0-spi"

static int
gdix51c0_bio_write (BIO *b, const char *buf, int len)
{
  Gdix51c0Tls *tls = BIO_get_data (b);
  GError *error = NULL;

  BIO_clear_retry_flags (b);
  if (!gdix51c0_spi_write (tls->dev, tls->spi_fd, GDIX51C0_PKT_READ /* 0xb0 */,
                        (const guint8 *) buf, (gsize) len, &error))
    {
      fp_warn ("gdix51c0: tls bio write failed: %s",
               error ? error->message : "?");
      g_clear_error (&error);
      return -1;
    }
  return len;
}

/* CKE record layout (TLS body 21 bytes):
 *   16 03 03 00 15            TLS handshake record header, len 21
 *   10 00 00 11               handshake type CKE, len 17
 *   00 0F                     PSK identity length 15
 *   <15 bytes>                "Client_identity"
 */
#define GDIX51C0_CKE_PREFIX_LEN 11
static const guint8 gdix51c0_cke_header[GDIX51C0_CKE_PREFIX_LEN] = {
  0x16, 0x03, 0x03, 0x00, 0x15, 0x10, 0x00, 0x00, 0x11, 0x00, 0x0f
};
static const guint8 gdix51c0_cke_identity[15] = "Client_identity";

static void
gdix51c0_patch_inbound (guint8 *buf, gsize len)
{
  /* Malformed empty ChangeCipherSpec: 14 03 03 00 00 00 -> 14 03 03 00 01 01 */
  if (len >= 6 &&
      buf[0] == 0x14 && buf[1] == 0x03 && buf[2] == 0x03 &&
      buf[3] == 0x00 && buf[4] == 0x00 && buf[5] == 0x00)
    {
      buf[4] = 0x01;
      buf[5] = 0x01;
      fp_dbg ("gdix51c0: patched malformed empty ChangeCipherSpec");
    }

  /* Truncated CKE identity -> restore "Client_identity". */
  gsize cke_full = GDIX51C0_CKE_PREFIX_LEN + sizeof (gdix51c0_cke_identity);
  if (len >= cke_full &&
      memcmp (buf, gdix51c0_cke_header, GDIX51C0_CKE_PREFIX_LEN) == 0 &&
      memcmp (buf + GDIX51C0_CKE_PREFIX_LEN, gdix51c0_cke_identity, sizeof (gdix51c0_cke_identity)) != 0)
    {
      memcpy (buf + GDIX51C0_CKE_PREFIX_LEN, gdix51c0_cke_identity, sizeof (gdix51c0_cke_identity));
      fp_dbg ("gdix51c0: restored truncated PSK identity in CKE");
    }
}

static int
gdix51c0_bio_read (BIO *b, char *buf, int len)
{
  Gdix51c0Tls *tls = BIO_get_data (b);

  BIO_clear_retry_flags (b);

  if (tls->rd_off >= tls->rd_len)
    {
      /* Refill from one SPI packet.  Wait for IRQ rise → read → fall. */
      GError *error = NULL;

      if (!gdix51c0_irq_wait (tls->irq_req, tls->irq_events, TRUE,
                           tls->irq_offset, GDIX51C0_BOOT_TIMEOUT_USEC,
                           "tls-read-rise", &error))
        {
          fp_warn ("gdix51c0: tls bio read rise: %s", error ? error->message : "?");
          g_clear_error (&error);
          return -1;
        }

      gsize plen = 0;
      guint8 *pkt = gdix51c0_spi_read (tls->dev, tls->spi_fd, &plen, &error);
      if (!pkt)
        {
          fp_warn ("gdix51c0: tls bio spi_read: %s", error ? error->message : "?");
          g_clear_error (&error);
          return -1;
        }

      gdix51c0_irq_wait (tls->irq_req, tls->irq_events, FALSE,
                      tls->irq_offset, GDIX51C0_BOOT_TIMEOUT_USEC,
                      "tls-read-fall", &error);
      g_clear_error (&error);

      gdix51c0_patch_inbound (pkt, plen);

      g_free (tls->rd_buf);
      tls->rd_buf = pkt;
      tls->rd_off = 0;
      tls->rd_len = plen;
    }

  gsize avail = tls->rd_len - tls->rd_off;
  gsize n = MIN ((gsize) len, avail);
  memcpy (buf, tls->rd_buf + tls->rd_off, n);
  tls->rd_off += n;
  return (int) n;
}

static long
gdix51c0_bio_ctrl (BIO *b, int cmd, long larg, void *parg)
{
  (void) b; (void) larg; (void) parg;
  switch (cmd)
    {
    case BIO_CTRL_FLUSH:
      return 1;
    default:
      return 0;
    }
}

static int
gdix51c0_bio_create (BIO *b)
{
  BIO_set_init (b, 1);
  BIO_set_data (b, NULL);
  return 1;
}

static int
gdix51c0_bio_destroy (BIO *b)
{
  if (!b)
    return 0;
  return 1;
}

static BIO_METHOD *
gdix51c0_bio_method (void)
{
  static BIO_METHOD *m = NULL;
  if (!m)
    {
      m = BIO_meth_new (BIO_TYPE_SOURCE_SINK, GDIX51C0_BIO_NAME);
      BIO_meth_set_write   (m, gdix51c0_bio_write);
      BIO_meth_set_read    (m, gdix51c0_bio_read);
      BIO_meth_set_ctrl    (m, gdix51c0_bio_ctrl);
      BIO_meth_set_create  (m, gdix51c0_bio_create);
      BIO_meth_set_destroy (m, gdix51c0_bio_destroy);
    }
  return m;
}

/* ---------------- PSK callback ---------------- */

static unsigned int
gdix51c0_psk_server_cb (SSL *ssl,
                     const char *identity,
                     unsigned char *out,
                     unsigned int max_len)
{
  Gdix51c0Tls *tls = SSL_get_app_data (ssl);

  if (!identity || g_strcmp0 (identity, GDIX51C0_PSK_IDENTITY) != 0)
    {
      fp_warn ("gdix51c0: tls: unexpected PSK identity %s", identity ? identity : "(null)");
      return 0;
    }
  if (!tls || tls->psk_len == 0 || tls->psk_len > max_len)
    return 0;
  memcpy (out, tls->psk, tls->psk_len);
  return (unsigned int) tls->psk_len;
}

/* ---------------- Public API ---------------- */

static GError *
gdix51c0_openssl_error (const char *what)
{
  unsigned long e = ERR_get_error ();
  char buf[256];
  if (e)
    ERR_error_string_n (e, buf, sizeof (buf));
  else
    g_strlcpy (buf, "unknown OpenSSL error", sizeof (buf));
  return g_error_new (G_IO_ERROR, G_IO_ERROR_FAILED, "gdix51c0: %s: %s", what, buf);
}

gboolean
gdix51c0_tls_init (Gdix51c0Tls *tls,
                FpDevice *dev,
                int spi_fd,
                struct gpiod_line_request      *irq_req,
                struct gpiod_edge_event_buffer *irq_events,
                unsigned int                    irq_offset,
                const guint8 *psk, gsize psk_len,
                GError **error)
{
  memset (tls, 0, sizeof (*tls));
  tls->dev        = dev;
  tls->spi_fd     = spi_fd;
  tls->irq_req    = irq_req;
  tls->irq_events = irq_events;
  tls->irq_offset = irq_offset;
  if (psk_len > sizeof (tls->psk))
    {
      g_set_error (error, G_IO_ERROR, G_IO_ERROR_INVALID_DATA,
                   "gdix51c0: PSK too long (%zu > %zu)", psk_len, sizeof (tls->psk));
      return FALSE;
    }
  memcpy (tls->psk, psk, psk_len);
  tls->psk_len = psk_len;

  tls->ctx = SSL_CTX_new (TLS_server_method ());
  if (!tls->ctx)
    {
      g_propagate_error (error, gdix51c0_openssl_error ("SSL_CTX_new"));
      return FALSE;
    }
  SSL_CTX_set_min_proto_version (tls->ctx, TLS1_2_VERSION);
  SSL_CTX_set_max_proto_version (tls->ctx, TLS1_2_VERSION);
  if (SSL_CTX_set_cipher_list (tls->ctx, "PSK-AES128-GCM-SHA256") != 1)
    {
      g_propagate_error (error, gdix51c0_openssl_error ("set_cipher_list"));
      return FALSE;
    }
  /* Tell OpenSSL to skip cert verification — we have no cert chain on
   * either side; PSK alone authenticates. */
  SSL_CTX_set_verify (tls->ctx, SSL_VERIFY_NONE, NULL);
  SSL_CTX_set_psk_server_callback (tls->ctx, gdix51c0_psk_server_cb);
  SSL_CTX_use_psk_identity_hint (tls->ctx, GDIX51C0_PSK_IDENTITY);

  tls->ssl = SSL_new (tls->ctx);
  if (!tls->ssl)
    {
      g_propagate_error (error, gdix51c0_openssl_error ("SSL_new"));
      return FALSE;
    }

  /* SSL_set_app_data is shorthand for ex_data slot 0 — that's how the
   * PSK callback recovers our Gdix51c0Tls (which holds the PSK). */
  SSL_set_app_data (tls->ssl, tls);

  tls->bio = BIO_new (gdix51c0_bio_method ());
  if (!tls->bio)
    {
      g_propagate_error (error, gdix51c0_openssl_error ("BIO_new"));
      return FALSE;
    }
  BIO_set_data (tls->bio, tls);
  SSL_set_bio (tls->ssl, tls->bio, tls->bio);
  /* tls->bio is now owned by ssl */
  SSL_set_accept_state (tls->ssl);
  return TRUE;
}

gboolean
gdix51c0_tls_handshake (Gdix51c0Tls *tls, GError **error)
{
  /* Kick the MCU to start its ClientHello. */
  static const guint8 d1_payload[] = { 0xd1, 0x03, 0x00, 0x00, 0x00, 0xd7 };
  if (!gdix51c0_spi_write (tls->dev, tls->spi_fd, GDIX51C0_PKT_WRITE,
                        d1_payload, sizeof (d1_payload), error))
    return FALSE;

  /* Drive accept until done.  The custom BIO blocks on IRQ, so a single
   * SSL_accept call should run the whole handshake to completion. */
  int rc = SSL_accept (tls->ssl);
  if (rc <= 0)
    {
      int ssl_err = SSL_get_error (tls->ssl, rc);
      g_propagate_error (error, gdix51c0_openssl_error ("SSL_accept"));
      fp_warn ("gdix51c0: SSL_accept rc=%d ssl_err=%d", rc, ssl_err);
      return FALSE;
    }
  fp_dbg ("gdix51c0: TLS handshake complete (cipher=%s)",
          SSL_get_cipher (tls->ssl));
  return TRUE;
}

guint8 *
gdix51c0_tls_decrypt_record (Gdix51c0Tls *tls,
                          const guint8 *record, gsize record_len,
                          gsize *out_len, GError **error)
{
  /* Splice the encrypted record into our read buffer and pull plaintext
   * via SSL_read.  TLS app records on this MCU are <16 KiB so a single
   * SSL_read call is enough. */
  g_free (tls->rd_buf);
  tls->rd_buf = g_memdup2 (record, record_len);
  tls->rd_off = 0;
  tls->rd_len = record_len;

  guint8 *plain = g_malloc (record_len);
  int n = SSL_read (tls->ssl, plain, (int) record_len);
  if (n <= 0)
    {
      g_free (plain);
      g_propagate_error (error, gdix51c0_openssl_error ("SSL_read"));
      return NULL;
    }
  if (out_len)
    *out_len = (gsize) n;
  return plain;
}

void
gdix51c0_tls_free (Gdix51c0Tls *tls)
{
  if (!tls) return;
  if (tls->ssl)
    {
      SSL_free (tls->ssl);
      tls->ssl = NULL;
      tls->bio = NULL; /* freed via SSL_free */
    }
  if (tls->ctx)
    {
      SSL_CTX_free (tls->ctx);
      tls->ctx = NULL;
    }
  g_clear_pointer (&tls->rd_buf, g_free);
}
