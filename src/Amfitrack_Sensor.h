//-----------------------------------------------------------------------------
//                              AMFITECH APS
//                          ALL RIGHTS RESERVED
//-----------------------------------------------------------------------------
#pragma once

//-----------------------------------------------------------------------------
// Section: Includes
//-----------------------------------------------------------------------------
#include "project_conf.h"

#include "../AmfitrackDeviceTypes.h"
#include "../lib/amfiprotapi/lib_AmfiProt_Amfitrack.hpp"

#include <chrono>
#include <cstdint>

#ifdef USE_USB
#include "hidapi.h"
#endif
//-----------------------------------------------------------------------------
// Section: Define
//-----------------------------------------------------------------------------
#ifndef MAX_NAME_LENGTH
#define MAX_NAME_LENGTH 64
#endif
//-----------------------------------------------------------------------------
// Section: Typedef
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Section: Macro
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Section: Variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Section: Class
//-----------------------------------------------------------------------------

class AMFITRACK_Sensor
{
  public:
	AMFITRACK_Sensor();
	explicit AMFITRACK_Sensor(uint8_t id);

	void reset();

  public:
	uint8_t deviceId;
	char name[MAX_NAME_LENGTH];
	uint32_t uuid[3];
	FW_t FW_Version;
	RF_t RF_Version;
	HW_t HW_Version;

	bool active;
	uint8_t hub_ID;

	uint16_t calcId;

	hid_device *_dev_handle;

	Pose_t pose;
	IMU_t imu;

#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
	std::chrono::steady_clock::time_point timestamp;
#endif

	// Deprecated!
	lib_AmfiProt_Amfitrack_Sensor_Measurement_t sensorMeasurement;

	uint32_t lastTimeSeenMs;
};
