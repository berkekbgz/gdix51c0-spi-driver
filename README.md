# GDIX51C0 SPI libfprint Driver

Out-of-tree injection repo for the Goodix `GDIX51C0` SPI fingerprint sensor.

## Structure

```text
drivers/gdix51c0/          GDIX51C0 SPI driver sources
sigfm/                     SIGFM C API used for matching small 64x80 images
meson-integration.patch    libfprint Meson integration patch
install.sh                 copies sources and applies the patch to libfprint
```

## Install Into libfprint

```bash
./install.sh /path/to/libfprint
meson setup /path/to/libfprint/builddir --reconfigure
ninja -C /path/to/libfprint/builddir
```

The driver requires `libgpiod >= 2.0`, `OpenSSL >= 3.0`, `gudev`, and OpenCV 4 libraries (`opencv_core`, `opencv_features2d`, `opencv_flann`, `opencv_imgproc`).

Runtime hardware overrides are available through `GDIX51C0_GPIOCHIP`, `GDIX51C0_IRQ_LINE`, `GDIX51C0_RESET_LINE`, and `GOODIX_TLS_PSK_HEX`.
