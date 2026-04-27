#!/usr/bin/env bash
# Inject the GDIX51C0 SPI driver into a libfprint source tree.

set -euo pipefail

LIBFPRINT_DIR="${1:?Usage: $0 /path/to/libfprint}"
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
PATCH_FILE="$SCRIPT_DIR/meson-integration.patch"

if [ ! -f "$LIBFPRINT_DIR/meson.build" ] || [ ! -f "$LIBFPRINT_DIR/libfprint/meson.build" ]; then
  printf 'Error: %s does not look like a libfprint source tree.\n' "$LIBFPRINT_DIR" >&2
  exit 1
fi

printf 'Installing GDIX51C0 driver into %s\n' "$LIBFPRINT_DIR"

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

printf '\nDone. Reconfigure and rebuild libfprint, for example:\n'
printf '  meson setup builddir --reconfigure\n'
printf '  ninja -C builddir\n'
