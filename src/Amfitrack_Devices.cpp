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
#include "Amfitrack_Devices.h"

#include "../lib/lib_log/lib_log.h"

#include <algorithm>
#include <cstring>
//-----------------------------------------------------------------------------
// Section: Define
//-----------------------------------------------------------------------------
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
// Section: Function prototypes
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Section: Functions
//-----------------------------------------------------------------------------

AMFITRACK_Devices &AMFITRACK_Devices::getInstance()
{
	static AMFITRACK_Devices instance;
	return instance;
}

AMFITRACK_Devices::AMFITRACK_Devices()
{
	setup_device_slots();
}

void AMFITRACK_Devices::reset_devices()
{
#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	setup_device_slots();
}

bool AMFITRACK_Devices::reset_sensor(uint8_t device_id)
{
	if (!is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	_sensors[device_id].reset();
	_sensors[device_id].deviceId = device_id;
	return true;
}

bool AMFITRACK_Devices::reset_source(uint8_t device_id)
{
	if (!is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	_sources[device_id].reset();
	_sources[device_id].deviceId = device_id;
	return true;
}

std::size_t AMFITRACK_Devices::device_count()
{
	return AMFITRACK_DEVICE_COUNT;
}

bool AMFITRACK_Devices::is_valid_device_id(uint8_t device_id)
{
	return (device_id < AMFITRACK_DEVICE_COUNT) &&
		   (device_id != AMFITRACK_BROADCAST_DEVICE_ID);
}

uint32_t AMFITRACK_Devices::get_time_ms()
{
	return static_cast<uint32_t>(
		std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now().time_since_epoch())
			.count());
}

bool AMFITRACK_Devices::get_sensor(uint8_t device_id, AMFITRACK_Sensor *sensor) const
{
	if ((sensor == nullptr) || !is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	*sensor = _sensors[device_id];
	return true;
}

bool AMFITRACK_Devices::get_source(uint8_t device_id, AMFITRACK_Source *source) const
{
	if ((source == nullptr) || !is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	*source = _sources[device_id];
	return true;
}

bool AMFITRACK_Devices::set(uint8_t device_id, bool isActive)
{
	if (!is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	if (_sensors[device_id].active != isActive || _sources[device_id].active != isActive)
	{
		_sensors[device_id].active = isActive;
		_sources[device_id].active = isActive;

		if (isActive)
		{
			LOG_I("Device %u connected!", device_id);
		}
		else
		{
			LOG_I("Device %u disconnected!", device_id);
		}
	}

	if (isActive)
	{
		update_last_seen(device_id, true, true);
	}

	return true;
}

bool AMFITRACK_Devices::set(uint8_t device_id, char const *name, uint8_t length)
{
	if (!is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	if ((name != nullptr) && (length > 0))
	{
		const std::size_t copy_len = std::min<std::size_t>(length, MAX_NAME_LENGTH - 1);
		std::memcpy(_sensors[device_id].name, name, copy_len);
		_sensors[device_id].name[copy_len] = '\0';
		std::memcpy(_sources[device_id].name, name, copy_len);
		_sources[device_id].name[copy_len] = '\0';
	}
	else
	{
		_sensors[device_id].name[0] = '\0';
		_sources[device_id].name[0] = '\0';
	}

	update_last_seen(device_id, true, true);
	return true;
}

bool AMFITRACK_Devices::set(uint8_t device_id, uint8_t hubId)
{
	if (!is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	_sensors[device_id].hub_ID = hubId;
	_sources[device_id].hub_ID = hubId;
	update_last_seen(device_id, true, true);
	return true;
}

bool AMFITRACK_Devices::set_hid(uint8_t device_id, hid_device *hidHandle, bool sensor)
{
	if (!is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	if (sensor)
	{
		_sensors[device_id]._dev_handle = hidHandle;
	}
	else
	{
		_sources[device_id]._dev_handle = hidHandle;
	}

	return true;
}

bool AMFITRACK_Devices::set(uint8_t device_id, Pose_t const &pose)
{
	if (!is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	_sensors[device_id].pose = pose;
	update_last_seen(device_id, true, false);
#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
	_sensors[device_id].timestamp = std::chrono::steady_clock::now();
#endif
	return true;
}

bool AMFITRACK_Devices::set(uint8_t device_id, IMU_t const &imu)
{
	if (!is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	_sensors[device_id].imu = imu;
	update_last_seen(device_id, true, false);
	return true;
}

bool AMFITRACK_Devices::set(uint8_t device_id, lib_AmfiProt_Amfitrack_Sensor_Measurement_t const &sensorMeasurement)
{
	if (!is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	_sensors[device_id].sensorMeasurement = sensorMeasurement;
	update_last_seen(device_id, true, false);
	return true;
}

bool AMFITRACK_Devices::set(uint8_t device_id, Current_t const &current)
{
	if (!is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	_sources[device_id].current = current;
	update_last_seen(device_id, false, true);
	return true;
}

bool AMFITRACK_Devices::set(uint8_t device_id, Frequency_t const &frequency)
{
	if (!is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	_sources[device_id].frequency = frequency;
	update_last_seen(device_id, false, true);
	return true;
}

bool AMFITRACK_Devices::set(uint8_t device_id, Voltage_t const &voltage)
{
	if (!is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	_sources[device_id].voltage = voltage;
	update_last_seen(device_id, false, true);
	return true;
}

bool AMFITRACK_Devices::set(uint8_t device_id, Calibration_t const &calibration)
{
	if (!is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	_sources[device_id].calibration = calibration;
	update_last_seen(device_id, false, true);
	return true;
}

void AMFITRACK_Devices::setup_device_slots()
{
	for (std::size_t device_id = 0; device_id < AMFITRACK_DEVICE_COUNT; device_id++)
	{
		_sensors[device_id].reset();
		_sensors[device_id].deviceId = static_cast<uint8_t>(device_id);

		_sources[device_id].reset();
		_sources[device_id].deviceId = static_cast<uint8_t>(device_id);
	}
}

void AMFITRACK_Devices::update_last_seen(uint8_t device_id, bool sensor, bool source)
{
	const uint32_t now = get_time_ms();

	if (_sensors[device_id].active != true && _sources[device_id].active != true)
	{
		LOG_I("Device %u connected!", device_id);
	}

	if (sensor)
	{
		_sensors[device_id].lastTimeSeenMs = now;
		_sensors[device_id].active = true;
	}

	if (source)
	{
		_sources[device_id].lastTimeSeenMs = now;
		_sources[device_id].active = true;
	}
}
