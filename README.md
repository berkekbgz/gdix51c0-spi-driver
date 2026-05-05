# GDIX51C0 SPI libfprint Driver

Out-of-tree injection repo for the Goodix `GDIX51C0` SPI fingerprint sensor.

## Structure

```text
drivers/gdix51c0/          GDIX51C0 SPI driver sources
sigfm/                     SIGFM C API used for matching small 64x80 images
meson-integration.patch    libfprint Meson integration patch
install.sh                 builds and installs the patched libfprint driver
```

## Build And Install

```bash
./install.sh
```

The script defaults to the bundled `./libfprint` tree, syncs the driver sources,
applies the Meson integration if needed, builds libfprint, installs the patched
library to `/opt/gdix51c0-libfprint`, writes the `fprintd` systemd override, and
restarts `fprintd`.

To use a different libfprint source tree:

```bash
./install.sh /path/to/libfprint
```

The driver requires `libgpiod >= 2.0`, `OpenSSL >= 3.0`, `gudev`, and OpenCV 4 libraries (`opencv_core`, `opencv_features2d`, `opencv_flann`, `opencv_imgproc`).

Runtime hardware overrides are available through `GDIX51C0_GPIOCHIP`, `GDIX51C0_IRQ_LINE`, `GDIX51C0_RESET_LINE`, and `GOODIX_TLS_PSK_HEX`.
