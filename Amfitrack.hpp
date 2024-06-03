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
#include <cstdlib>
#include <iostream>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <vector>
#include <memory>
#include <thread>
#include "lib/amfiprotapi/lib_AmfiProt_API.hpp"
#pragma pack(8)
namespace AMFITRACK_API_LIB
{

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------
#define AMFITRACK_MAX_NAME_LENGTH 64
#define AMFITRACK_MAX_NUMBER_OF_DEVICES 254
#define AMFITRACK_UUID_LENGTH 24
#define MEDABILITY_SENSOR_SLOT_COUNT 4
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
	void initialize_amfitrack(void);

	/* Starts the main thread, that reads data from all connected devices */
	void start_amfitrack_task(void);

	/* Stops the main thread */
	void stop_amfitrack_task(void);

	void setDeviceName(uint8_t DeviceID, char* name, uint8_t length);
	std::string getDeviceName(uint8_t DeviceID);

	void setDeviceActive(uint8_t DeviceID);
	/* Return if a device is connected */
	bool getDeviceActive(uint8_t DeviceID);

	void setDevicePose(uint8_t DeviceID, lib_AmfiProt_Amfitrack_Pose_t Pose);
	/* Get the pose for a specific device */
	void getDevicePose(uint8_t DeviceID, lib_AmfiProt_Amfitrack_Pose_t* Pose);

	void setSensorSlot(uint8_t slot, uint8_t DeviceID);
	uint8_t getSensorInSlot(uint8_t slot);

	void getDeviceTemperature(uint8_t DeviceID, float* temp);
	void setDeviceTemperature(uint8_t DeviceID, float temp);

	void setDeviceUUID(uint8_t DeviceID, uint32_t UUID[3]);
	std::string getDeviceUUID(uint8_t DeviceID);
	uint32_t getDeviceUUIDShort(uint8_t DeviceID);

	bool getSlotDeviceChanged(uint8_t DeviceID);
	void resetSlotDeviceChanged(uint8_t DeviceID);


	std::vector<uint8_t> getSensorIDs();
	uint32_t getRFHubSerial();
	uint32_t getEMFSourceSerial();
	uint8_t getEMFSourceID();

	std::vector<uint8_t>  getActiveDevices();

private:

	
	char Name[AMFITRACK_MAX_NUMBER_OF_DEVICES][AMFITRACK_MAX_NAME_LENGTH]; // Array of character arrays to store device names
	bool DeviceActive[AMFITRACK_MAX_NUMBER_OF_DEVICES];
	float Temperature[AMFITRACK_MAX_NUMBER_OF_DEVICES];
	char DeviceUUID[AMFITRACK_MAX_NUMBER_OF_DEVICES][AMFITRACK_UUID_LENGTH];
	uint32_t DeviceUUID_Number[AMFITRACK_MAX_NUMBER_OF_DEVICES];
	time_t DeviceLastTimeSeen[AMFITRACK_MAX_NUMBER_OF_DEVICES];


	uint8_t DeviceSlot[MEDABILITY_SENSOR_SLOT_COUNT];
	bool SlotDeviceChanged[MEDABILITY_SENSOR_SLOT_COUNT];

	lib_AmfiProt_Amfitrack_Pose_t Position[AMFITRACK_MAX_NUMBER_OF_DEVICES];

	uint8_t id_rfhub;
	uint8_t id_emfsource;
	std::vector<uint8_t> ids_sensor;

	static void background_amfitrack_task(AMFITRACK*);
	void checkDeviceDisconnected(uint8_t DeviceID);

	AMFITRACK();
	~AMFITRACK();
};
}


#endif //AMFITRACK_DEVICES_HPP_