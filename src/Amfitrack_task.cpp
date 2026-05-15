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

#include "../lib/amfiprotapi/lib_AmfiProt_API.hpp"
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

//-----------------------------------------------------------------------------
// Section: Function prototypes
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Section: Functions
//-----------------------------------------------------------------------------
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
}
