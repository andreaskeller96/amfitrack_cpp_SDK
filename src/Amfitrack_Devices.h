//-----------------------------------------------------------------------------
//                              AMFITECH APS
//                          ALL RIGHTS RESERVED
//
// D// $URL: $
// $Rev: $
// $Date: $
// $Author: $
//
// Description
// TODO Write a description here
//
//-----------------------------------------------------------------------------
#pragma once

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <string.h>
#include <chrono>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "project_conf.h"
#include "lib_AmfiProt_API.hpp"
#include "../AmfitrackDeviceTypes.h"
#ifdef USE_USB
#include "hidapi.h"
#endif
#ifdef USE_THREAD_BASED
#include <mutex>
#endif // USE_THREAD_BASED

#define MAX_NAME_LENGTH 64
#define MAX_NUMBER_OF_DEVICES 254

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

	uint16_t calcId;

	hid_device *_dev_handle;

	Pose_t pose;
	IMU_t imu;

#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
	std::chrono::steady_clock::time_point sensorTimestamp;
#endif

	uint32_t lastTimeSeenMs;
};

class AMFITRACK_Source
{
  public:
	AMFITRACK_Source();
	explicit AMFITRACK_Source(uint8_t id);

	void reset();

  public:
	uint8_t deviceId;
	char name[MAX_NAME_LENGTH];
	uint32_t uuid[3];
	FW_t FW_Version;
	RF_t RF_Version;
	HW_t HW_Version;

	bool active;

	hid_device *_dev_handle;

	Current_t current;
	Frequency_t frequency;
	Voltage_t voltage;
	Calibration_t calibration;

	uint32_t lastTimeSeenMs;
};
