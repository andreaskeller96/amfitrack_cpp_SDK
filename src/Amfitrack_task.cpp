//-----------------------------------------------------------------------------
//
//                              AMFITECH APS
//
//                          ALL RIGHTS RESERVED
//
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Section: Includes
//-----------------------------------------------------------------------------
#include "Amfitrack_task.h"

#include "lib_AmfiProt_API.hpp"
#include "Amfitrack_Devices.h"

#include <chrono>
#include <cstddef>
#include <mutex>
//-----------------------------------------------------------------------------
// Section: Define
//-----------------------------------------------------------------------------
#define DISCONNECT_TIMEOUT 5000
//-----------------------------------------------------------------------------
// Section: Typedef
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Section: Macro
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Section: Variables
//-----------------------------------------------------------------------------
static constexpr std::chrono::seconds kKeepAlivePingInterval(2);
static AmfiProt_API *amfiprot_api = nullptr;

static std::chrono::steady_clock::time_point lastKeepAlivePingTime{};
static std::mutex keepAlivePingMutex;

static constexpr std::chrono::seconds kGetMissingInfoInterval(2);
amfitrack_task::missingInfo_t amfitrack_task::missingInfo = amfitrack_task::missingInfo_t::missingInfo_FW;
static std::chrono::steady_clock::time_point lastGetMissingInfoTime{};
static std::mutex getMissingInfoMutex;
//-----------------------------------------------------------------------------
// Section: Function prototypes
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Section: Functions
//-----------------------------------------------------------------------------
void amfitrack_task::getVersion(uint8_t deviceID, uint8_t _version)
{
	lib_AmfiProt_FirmwareVersionPerIDRequest payload;
	payload.payloadID = lib_AmfiProt_PayloadID_RequestFirmwareVersionPerID;
	payload.processorID = _version;
	amfiprot_api->queue_frame(&payload, sizeof(payload), libAmfiProt_PayloadType_Common, lib_AmfiProt_packetType_NoAck, deviceID);
}

void amfitrack_task::getName(uint8_t deviceID)
{
	uint8_t payload = lib_AmfiProt_PayloadID_RequestDeviceName;
	amfiprot_api->queue_frame(&payload, sizeof(payload), libAmfiProt_PayloadType_Common, lib_AmfiProt_packetType_NoAck, deviceID);
}

void amfitrack_task::getMissingInfo()
{
	const auto now = std::chrono::steady_clock::now();
	{
		const std::lock_guard<std::mutex> lock(getMissingInfoMutex);
		if (lastGetMissingInfoTime == std::chrono::steady_clock::time_point{} ||
			now - lastGetMissingInfoTime >= kGetMissingInfoInterval)
		{
			for (uint8_t i = 0; i < AMFITRACK_DEVICE_COUNT; i++)
			{
				AMFITRACK_Sensor sensor;
				AMFITRACK_Devices::getInstance().get_sensor(i, &sensor);
				if (!sensor.active)
					continue;

				switch (missingInfo)
				{
					case amfitrack_task::missingInfo_t::missingInfo_FW:
						if (sensor.FW_Version.Major == 0)
						{
							getVersion(i, AMFITRACK_FW_VERSION_ID);
						}
						break;
					case amfitrack_task::missingInfo_t::missingInfo_RF:
						if (sensor.RF_Version.Major == 0)
						{
							getVersion(i, AMFITRACK_RF_VERSION_ID);
						}
						break;
					case amfitrack_task::missingInfo_t::missingInfo_HW:
						if (sensor.HW_Version.Generation == 0)
						{
							getVersion(i, AMFITRACK_HW_VERSION_ID);
						}
						break;
					case amfitrack_task::missingInfo_t::missingInfo_Name:
						if (sensor.name[0] == 0x00)
						{
							getName(i);
						}
						break;
				}
			}
			switch (missingInfo)
			{
				case amfitrack_task::missingInfo_t::missingInfo_FW:
					missingInfo = amfitrack_task::missingInfo_t::missingInfo_RF;
					break;
				case amfitrack_task::missingInfo_t::missingInfo_RF:
					missingInfo = amfitrack_task::missingInfo_t::missingInfo_HW;
					break;
				case amfitrack_task::missingInfo_t::missingInfo_HW:
					missingInfo = amfitrack_task::missingInfo_t::missingInfo_Name;
					break;
				case amfitrack_task::missingInfo_t::missingInfo_Name:
					missingInfo = amfitrack_task::missingInfo_t::missingInfo_FW;
					break;
			}
			lastGetMissingInfoTime = now;
		}
	}
}

void amfitrack_task::keepAlivePing()
{
	uint8_t payload = lib_AmfiProt_PayloadID_RequestDeviceID;
	amfiprot_api->queue_frame(&payload, sizeof(payload), libAmfiProt_PayloadType_Common, lib_AmfiProt_packetType_NoAck, lib_AmfiProt_destination_Broadcast);
}

void amfitrack_task::checkDisconnected()
{
	AMFITRACK_Devices &devices = AMFITRACK_Devices::getInstance();
	const uint32_t now = AMFITRACK_Devices::get_time_ms();

	for (std::size_t device_index = 0; device_index < AMFITRACK_Devices::device_count(); device_index++)
	{
		const auto device_id = static_cast<uint8_t>(device_index);
		AMFITRACK_Sensor _sensor;
		devices.get_sensor(device_id, &_sensor);
		if (_sensor.active && ((now - _sensor.lastTimeSeenMs) > DISCONNECT_TIMEOUT))
		{
			devices.set(device_id, false);
		}

		AMFITRACK_Source _source;
		devices.get_source(device_id, &_source);
		if (_source.active && ((now - _source.lastTimeSeenMs) > DISCONNECT_TIMEOUT))
		{
			devices.set(device_id, false);
		}
	}
}

void amfitrack_task::init()
{
	const std::lock_guard<std::mutex> lock(keepAlivePingMutex);
	amfiprot_api = &AmfiProt_API::getInstance();
	lastKeepAlivePingTime = {};
	lastGetMissingInfoTime = {};
	const auto now = std::chrono::steady_clock::now();
	lastKeepAlivePingTime = now;
	lastGetMissingInfoTime = now;
}

void amfitrack_task::run()
{
	const auto now = std::chrono::steady_clock::now();
	{
		const std::lock_guard<std::mutex> lock(keepAlivePingMutex);
		if (amfiprot_api != nullptr &&
			(lastKeepAlivePingTime == std::chrono::steady_clock::time_point{} ||
			 now - lastKeepAlivePingTime >= kKeepAlivePingInterval))
		{
			keepAlivePing();
			lastKeepAlivePingTime = now;
		}
	}

	checkDisconnected();
	getMissingInfo();
}
