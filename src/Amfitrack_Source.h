//-----------------------------------------------------------------------------
//                              AMFITECH APS
//                          ALL RIGHTS RESERVED
//-----------------------------------------------------------------------------
#pragma once
//-----------------------------------------------------------------------------
// Section: Includes
//-----------------------------------------------------------------------------
#include "../AmfitrackDeviceTypes.h"
#include "HID_Monitor.h"

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
	uint8_t hub_ID;

	hid_device *_dev_handle;

	Current_t current;
	Frequency_t frequency;
	Voltage_t voltage;
	Calibration_t calibration;

	uint32_t lastTimeSeenMs;
};
