//-----------------------------------------------------------------------------
//                              AMFITECH APS
//                          ALL RIGHTS RESERVED
//
// $URL: $
// $Rev: $
// $Date: $
// $Author: $
//
// Description
// TODO Write a description here
//
//-----------------------------------------------------------------------------

#ifndef AMFITRACK_DEVICES_HPP_
#define AMFITRACK_DEVICES_HPP_

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <string.h>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "src/project_conf.h"
#include "lib/amfiprotapi/lib_AmfiProt_API.hpp"


//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------
#define MAX_NAME_LENGTH 64
#define MAX_NUMBER_OF_DEVICES 254

//-----------------------------------------------------------------------------
// Type declarations
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Variables and constants
//-----------------------------------------------------------------------------
class AMFITRACK
{
public:
	static AMFITRACK& getInstance()
	{
		static AMFITRACK instance;
		return instance;
	}

	/* Call first to initialize USB and connect to devices */
	void initialize_amfitrack();

	/* Starts the main thread, that reads data from all connected devices */
	void start_amfitrack_task(void);

	/* Stops the main thread */
	void stop_amfitrack_task(void);

    void amfitrack_main_loop(void);

	void setDeviceName(uint8_t DeviceID, char* name, uint8_t length);

	void setConfiguration(uint8_t DeviceID, uint32_t UID, lib_Generic_Parameter_Value_t parameter);

	void setDeviceActive(uint8_t DeviceID);
	/* Return if a device is connected */
	bool getDeviceActive(uint8_t DeviceID);

	void setDevicePose(uint8_t DeviceID, lib_AmfiProt_Amfitrack_Pose_t Pose);
	/* Get the pose for a specific device */
	void getDevicePose(uint8_t DeviceID, lib_AmfiProt_Amfitrack_Pose_t* Pose);

	void setSensorMeasurements(uint8_t DeviceID,lib_AmfiProt_Amfitrack_Sensor_Measurement_t SensorMeasurement);

	void getSensorMeasurements(uint8_t DeviceID,lib_AmfiProt_Amfitrack_Sensor_Measurement_t* SensorMeasurement);

    void setDeviceBatterySOC(uint8_t DeviceID, uint8_t SOC);
    uint8_t getDeviceBatterySOC(uint8_t DeviceID);

#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
	void setSensorTimestamp(uint8_t DeviceID, std::chrono::steady_clock::time_point time_stamp);

	void getSensorTimestamp(uint8_t DeviceID, std::chrono::steady_clock::time_point* time_stamp);
#endif

private:
	char Name[MAX_NUMBER_OF_DEVICES][MAX_NAME_LENGTH]; // Array of character arrays to store device names
	bool DeviceActive[MAX_NUMBER_OF_DEVICES];
#ifdef USE_ACTIVE_DEVICE_HANDLING
	time_t DeviceLastTimeSeen[MAX_NUMBER_OF_DEVICES];
#endif

	lib_AmfiProt_Amfitrack_Pose_t Position[MAX_NUMBER_OF_DEVICES];
	lib_AmfiProt_Amfitrack_Sensor_Measurement_t SensorMeasurements[MAX_NUMBER_OF_DEVICES];
    uint8_t Battery_SOC[MAX_NUMBER_OF_DEVICES];
	
	static void background_amfitrack_task(AMFITRACK*);
	void checkDeviceDisconnected(uint8_t DeviceID);

	AMFITRACK();
	~AMFITRACK();
#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
	std::chrono::steady_clock::time_point SensorTimestamps[MAX_NUMBER_OF_DEVICES];
#endif
};




#endif //AMFITRACK_DEVICES_HPP_
