#!/bin/bash
# Build the bootloader for each base FC and copy it to the base board
# and to every derived board found as hwdef/<FC>-*
set -e

# base FCs to build the bootloader for
BOARDS=("AP6" "AP6m")

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"
HWDEF_DIR="$ROOT/libraries/AP_HAL_ChibiOS/hwdef"
BL_DIR="$ROOT/Tools/bootloaders"

cd "$ROOT"

for board in "${BOARDS[@]}"; do
    if [ ! -f "$HWDEF_DIR/$board/hwdef-bl.dat" ]; then
        echo "ERROR: $HWDEF_DIR/$board/hwdef-bl.dat not found" >&2
        exit 1
    fi

    echo "=== Building bootloader for $board ==="
    ./waf configure --board "$board" --bootloader
    ./waf bootloader
    src="build/$board/bin/AP_Bootloader.bin"

    cp "$src" "$BL_DIR/${board}_bl.bin"
    echo "  -> ${board}_bl.bin"

    # derived boards: <board>-*, only real board dirs (with hwdef.dat)
    for dir in "$HWDEF_DIR/$board"-*/; do
        [ -f "$dir/hwdef.dat" ] || continue
        name="$(basename "$dir")"
        cp "$src" "$BL_DIR/${name}_bl.bin"
        echo "  -> ${name}_bl.bin"
    done

    echo "$board bootloaders updated"
done
