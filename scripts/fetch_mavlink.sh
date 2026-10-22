#!/bin/bash
# Fetch the MAVLink C library (common dialect) into src/mavlink.
#
# Usage:
#   ./scripts/fetch_mavlink.sh [dest]      # dest defaults to src/mavlink
#
# The upstream library is MIT licensed and is not vendored in this repo.
set -e

COMMIT="${MAVLINK_COMMIT:-8c63a49}"
DEST="${1:-src/mavlink}"

TMP="$(mktemp -d)"
trap 'rm -rf "$TMP"' EXIT

git clone --filter=blob:none --no-checkout https://github.com/mavlink/c_library_v2.git "$TMP/c_library_v2"
git -C "$TMP/c_library_v2" checkout "$COMMIT"

mkdir -p "$DEST"
cp -r "$TMP/c_library_v2/common" "$DEST/"
cp "$TMP/c_library_v2"/*.h "$DEST/"

echo "MAVLink c_library_v2 @ $COMMIT -> $DEST"
ls "$DEST"
