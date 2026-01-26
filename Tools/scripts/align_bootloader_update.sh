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
