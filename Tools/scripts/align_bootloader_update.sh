#!/bin/bash
set -e
# Generate bootloaders
#TODO: horrible, use loop

# Align AP6m
./waf configure --board AP6m --bootloader
./waf bootloader
src="build/AP6m/bin/AP_Bootloader.bin"

# copy
cp $src "Tools/bootloaders/AP6m_bl.bin"
cp $src "Tools/bootloaders/AP6m-M3_bl.bin"
cp $src "Tools/bootloaders/AP6m-M3-OF_bl.bin"
cp $src "Tools/bootloaders/AP6m-M3-A10_bl.bin"
cp $src "Tools/bootloaders/AP6m-MR25_bl.bin"
cp $src "Tools/bootloaders/AP6m-MR25-A10_bl.bin"
cp $src "Tools/bootloaders/AP6m-MR25-no-gps_bl.bin"
echo "AP6m bootloaders updated"

# Align AG1, SDB1 and PCU3. Only the base boards carry a hwdef-bl.dat; the
# -MT40 airframe variants have none of their own. waf looks up
# Tools/bootloaders/<BOARD>_bl.bin under the exact board name being built, so
# each variant needs its own copy or it silently gets no _with_bl.hex.
for b in AG1 SDB1 PCU3; do
    ./waf configure --board $b --bootloader
    ./waf bootloader
    src="build/$b/bin/AP_Bootloader.bin"
    cp $src "Tools/bootloaders/${b}_bl.bin"
    cp $src "Tools/bootloaders/${b}-MT40_bl.bin"
    echo "$b bootloaders updated"
done
