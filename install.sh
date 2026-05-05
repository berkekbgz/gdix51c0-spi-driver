#!/usr/bin/env bash
# Build and install the GDIX51C0 SPI libfprint driver.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
LIBFPRINT_DIR="${1:-$SCRIPT_DIR/libfprint}"
BUILDDIR="${BUILDDIR:-$LIBFPRINT_DIR/builddir}"
INSTALL_DIR="${INSTALL_DIR:-/opt/gdix51c0-libfprint}"
FPRINTD_DROPIN_DIR="${FPRINTD_DROPIN_DIR:-/etc/systemd/system/fprintd.service.d}"
FPRINTD_DROPIN="$FPRINTD_DROPIN_DIR/override.conf"
GOODIX_TLS_PSK_HEX="${GOODIX_TLS_PSK_HEX:-3be6be9b57c3e7686c1fc8d7ffb4cd4374358d7229cf789a8e9147ec15c66f43}"
PATCH_FILE="$SCRIPT_DIR/meson-integration.patch"

usage() {
  printf 'Usage: %s [path/to/libfprint]\n' "$0"
  printf '\nDefaults to the bundled libfprint tree at %s/libfprint.\n' "$SCRIPT_DIR"
  printf 'Environment overrides: BUILDDIR, INSTALL_DIR, FPRINTD_DROPIN_DIR, GOODIX_TLS_PSK_HEX.\n'
}

run_root() {
  if [ "${EUID:-$(id -u)}" -eq 0 ]; then
    "$@"
  else
    sudo "$@"
  fi
}

if [ "${1:-}" = "-h" ] || [ "${1:-}" = "--help" ]; then
  usage
  exit 0
fi

if [ ! -f "$LIBFPRINT_DIR/meson.build" ] || [ ! -f "$LIBFPRINT_DIR/libfprint/meson.build" ]; then
  printf 'Error: %s does not look like a libfprint source tree.\n' "$LIBFPRINT_DIR" >&2
  usage >&2
  exit 1
fi

printf 'Installing GDIX51C0 driver sources into %s\n' "$LIBFPRINT_DIR"

install -d "$LIBFPRINT_DIR/libfprint/drivers/gdix51c0"
cp -v "$SCRIPT_DIR/drivers/gdix51c0/"*.c "$LIBFPRINT_DIR/libfprint/drivers/gdix51c0/"
cp -v "$SCRIPT_DIR/drivers/gdix51c0/"*.h "$LIBFPRINT_DIR/libfprint/drivers/gdix51c0/"

install -d "$LIBFPRINT_DIR/libfprint/sigfm"
cp -v "$SCRIPT_DIR/sigfm/sigfm.cpp" \
      "$SCRIPT_DIR/sigfm/sigfm.hpp" \
      "$SCRIPT_DIR/sigfm/binary.hpp" \
      "$SCRIPT_DIR/sigfm/img-info.hpp" \
      "$LIBFPRINT_DIR/libfprint/sigfm/"

if git -C "$LIBFPRINT_DIR" apply --check "$PATCH_FILE"; then
  git -C "$LIBFPRINT_DIR" apply "$PATCH_FILE"
  printf 'Applied %s\n' "$PATCH_FILE"
elif grep -q "'gdix51c0'" "$LIBFPRINT_DIR/meson.build" && \
     grep -q "libsigfm" "$LIBFPRINT_DIR/libfprint/meson.build"; then
  printf 'Meson integration already appears to be applied.\n'
else
  printf 'Error: could not apply %s. Check the libfprint version or apply it manually.\n' "$PATCH_FILE" >&2
  exit 1
fi

printf '\nConfiguring libfprint builddir at %s\n' "$BUILDDIR"
if [ -f "$BUILDDIR/meson-private/coredata.dat" ]; then
  (cd "$LIBFPRINT_DIR" && meson setup "$BUILDDIR" --reconfigure)
else
  if [ -e "$BUILDDIR" ]; then
    STALE_BUILDDIR="$BUILDDIR.stale.$(date +%Y%m%d%H%M%S)"
    printf 'Existing builddir is not a valid Meson builddir; moving it to %s\n' "$STALE_BUILDDIR"
    mv "$BUILDDIR" "$STALE_BUILDDIR"
  fi

  (cd "$LIBFPRINT_DIR" && meson setup "$BUILDDIR")
fi

printf '\nBuilding libfprint\n'
meson compile -C "$BUILDDIR"

BUILT_LIB="$BUILDDIR/libfprint/libfprint-2.so.2.0.0"
if [ ! -f "$BUILT_LIB" ]; then
  printf 'Error: expected built library not found: %s\n' "$BUILT_LIB" >&2
  exit 1
fi

printf '\nInstalling patched libfprint to %s\n' "$INSTALL_DIR"
run_root install -d "$INSTALL_DIR"
run_root install -m 0755 "$BUILT_LIB" "$INSTALL_DIR/libfprint-2.so.2.0.0"
run_root ln -sfn libfprint-2.so.2.0.0 "$INSTALL_DIR/libfprint-2.so.2"
run_root ln -sfn libfprint-2.so.2.0.0 "$INSTALL_DIR/libfprint-2.so"

printf '\nInstalling fprintd systemd override at %s\n' "$FPRINTD_DROPIN"
DROPIN_TMP="$(mktemp)"
trap 'rm -f "$DROPIN_TMP"' EXIT
cat > "$DROPIN_TMP" <<EOF
[Service]
Environment=GOODIX_TLS_PSK_HEX=$GOODIX_TLS_PSK_HEX
Environment=LD_LIBRARY_PATH=$INSTALL_DIR
Environment=G_MESSAGES_DEBUG=all
DeviceAllow=/dev/gpiochip0 rw
EOF
run_root install -d "$FPRINTD_DROPIN_DIR"
run_root install -m 0644 "$DROPIN_TMP" "$FPRINTD_DROPIN"

MODULES_LOAD_SRC="$SCRIPT_DIR/system/modules-load.d/gdix51c0-spidev.conf"
MODULES_LOAD_DST="/etc/modules-load.d/gdix51c0-spidev.conf"
UDEV_RULE_SRC="$SCRIPT_DIR/system/udev/91-gdix51c0-spidev.rules"
UDEV_RULE_DST="/etc/udev/rules.d/91-gdix51c0-spidev.rules"

printf '\nInstalling spidev autoload at %s\n' "$MODULES_LOAD_DST"
run_root install -d /etc/modules-load.d
run_root install -m 0644 "$MODULES_LOAD_SRC" "$MODULES_LOAD_DST"

printf 'Installing GDIX51C0 spidev bind rule at %s\n' "$UDEV_RULE_DST"
run_root install -d /etc/udev/rules.d
run_root install -m 0644 "$UDEV_RULE_SRC" "$UDEV_RULE_DST"

if command -v udevadm >/dev/null 2>&1; then
  printf 'Reloading udev rules and triggering re-bind\n'
  run_root udevadm control --reload-rules
  run_root modprobe spidev || true
  run_root udevadm trigger --subsystem-match=spi --action=change
  run_root udevadm settle
fi

if command -v systemctl >/dev/null 2>&1; then
  printf '\nReloading systemd and restarting fprintd\n'
  run_root systemctl daemon-reload
  run_root systemctl restart fprintd.service
else
  printf '\nDone. systemctl was not found; restart fprintd manually.\n'
fi

printf '\nDone. Test with:\n'
printf '  fprintd-list "$USER"\n'
printf '  fprintd-verify\n'
