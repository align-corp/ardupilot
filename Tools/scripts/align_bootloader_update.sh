#!/bin/bash
set -e
# Generate bootloaders
#TODO: horrible, use loop

# Align AP6m
./waf configure --board AP6m --bootloader
./waf bootloader
src="build/AP6m/bin/AP_Bootloader.bin"
./waf configure --board AP6m-MR25 --bootloader
./waf bootloader
src2="build/AP6m-MR25/bin/AP_Bootloader.bin"

# copy
cp $src "Tools/bootloaders/AP6m_bl.bin"
cp $src "Tools/bootloaders/AP6m-M3_bl.bin"
cp $src "Tools/bootloaders/AP6m-M3-OF_bl.bin"
cp $src "Tools/bootloaders/AP6m-M3-A10_bl.bin"
cp $src "Tools/bootloaders/AP6m-M460_bl.bin"
cp $src "Tools/bootloaders/AP6m-M460-A10_bl.bin"
cp $src "Tools/bootloaders/AP6m-M490_bl.bin"
cp $src "Tools/bootloaders/AP6m-M490-A10_bl.bin"
cp $src "Tools/bootloaders/AP6m-M450_bl.bin"
cp $src "Tools/bootloaders/AP6m-M450-A10_bl.bin"
cp $src2 "Tools/bootloaders/AP6m-MR25_bl.bin"
cp $src2 "Tools/bootloaders/AP6m-MR25-A10_bl.bin"
cp $src2 "Tools/bootloaders/AP6m-MR25-no-gps_bl.bin"
echo "Update bootloader for AP6m"
