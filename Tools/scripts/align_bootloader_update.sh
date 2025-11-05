#!/bin/bash
set -e
# Generate bootloaders

# Align AP6m
src="build/AP6m/bin/AP_Bootloader.bin"
./waf configure --board AP6m --bootloader
./waf bootloader
if [ -f $src ]; then
    cp $src "Tools/bootloaders/AP6m_bl.bin"
    cp $src "Tools/bootloaders/AP6m-M3_bl.bin"
    cp $src "Tools/bootloaders/AP6m-M3-OF_bl.bin"
    cp $src "Tools/bootloaders/AP6m-M3-A10_bl.bin"
    echo "Update bootloader for AP6m"
else
    echo "Bootloader doesn't exist"
    exit 2
fi
