//                              AMFITECH APS
//                          ALL RIGHTS RESERVED
//-----------------------------------------------------------------------------
#pragma once

/* Depending on what version of the firmware is being used change this.
 * Version is written as single number 300 which is the same as 3.0.0.x the last x is not being used for this firmware version.
 * Source measurement package changed from firmware 300 to 301. */
#define FIRMWARE_VERSION 301

/* Use if library should handle device connection and read/write to devices.
 * If disabled library can be used to deserialize and do the package handling alone */
#define USE_ACTIVE_DEVICE_HANDLING

/* Use if a USB connection is needed.
 * Is using HIDAPI and runs in windows */
#define USE_USB

/* Use if a thread is needed to handle USB and process data */
#define USE_THREAD_BASED
