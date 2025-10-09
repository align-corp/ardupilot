#pragma once

#ifndef FORCE_VERSION_H_INCLUDE
#error version.h should never be included directly. You probably want to include AP_Common/AP_FWVersion.h
#endif

#include "ap_version.h"

#define THISFIRMWARE "ArduCopter V4.5.7 Align V3.5.2"

// the following line is parsed by the autotest scripts
#define FIRMWARE_VERSION 4,5,7,FIRMWARE_VERSION_TYPE_OFFICIAL

// ArduPilot version
#define FW_MAJOR 4
#define FW_MINOR 5
#define FW_PATCH 7
#define FW_TYPE FIRMWARE_VERSION_TYPE_OFFICIAL

// Align version 
#define MIDDLE_MAJOR 3
#define MIDDLE_MINOR 5
#define MIDDLE_PATCH 2
#define MIDDLE_TYPE FIRMWARE_VERSION_TYPE_DEV

#include <AP_Common/AP_FWVersionDefine.h>
