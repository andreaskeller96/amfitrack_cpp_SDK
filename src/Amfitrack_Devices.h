//-----------------------------------------------------------------------------
//                              AMFITECH APS
//                          ALL RIGHTS RESERVED
//-----------------------------------------------------------------------------
#pragma once

//-----------------------------------------------------------------------------
// Section: Includes
//-----------------------------------------------------------------------------
#include "Amfitrack_Sensor.h"
#include "Amfitrack_Source.h"

#include <chrono>
#include <cstddef>
#include <cstdint>

#ifdef USE_THREAD_BASED
#include <mutex>
#endif
//-----------------------------------------------------------------------------
// Section: Define
//-----------------------------------------------------------------------------
#define AMFITRACK_DEVICE_COUNT 255
#define AMFITRACK_BROADCAST_DEVICE_ID 255U
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

class AMFITRACK_Devices
{
  public:
	static AMFITRACK_Devices &getInstance();

	void reset_devices();
	bool reset_sensor(uint8_t device_id);
	bool reset_source(uint8_t device_id);

	static std::size_t device_count();
	static bool is_valid_device_id(uint8_t device_id);
	static uint32_t get_time_ms();

	bool get_sensor(uint8_t device_id, AMFITRACK_Sensor *sensor) const;
	bool get_source(uint8_t device_id, AMFITRACK_Source *source) const;

	bool set(uint8_t device_id, bool isActive);
	bool set(uint8_t device_id, char const *name, uint8_t length);
	bool set(uint8_t device_id, uint8_t hubId);
	bool set_hid(uint8_t device_id, hid_device *hidHandle, bool sensor);
	bool set(uint8_t device_id, DeviceConfig_t const &config);

	bool set(uint8_t device_id, Pose_t const &pose);
	bool set(uint8_t device_id, IMU_t const &imu);
	// Deprecated!
	bool set(uint8_t device_id, lib_AmfiProt_Amfitrack_Sensor_Measurement_t const &sensorMeasurement);

	bool set(uint8_t device_id, Current_t const &current);
	bool set(uint8_t device_id, Frequency_t const &frequency);
	bool set(uint8_t device_id, Voltage_t const &voltage);
	bool set(uint8_t device_id, Calibration_t const &calibration);

  private:
	AMFITRACK_Devices();
	~AMFITRACK_Devices() = default;

	AMFITRACK_Devices(AMFITRACK_Devices const &) = delete;
	AMFITRACK_Devices &operator=(AMFITRACK_Devices const &) = delete;

	void setup_device_slots();
	void update_last_seen(uint8_t device_id, bool sensor, bool source);

#ifdef USE_THREAD_BASED
	mutable std::mutex _mutex;
#endif

	AMFITRACK_Sensor _sensors[AMFITRACK_DEVICE_COUNT];
	AMFITRACK_Source _sources[AMFITRACK_DEVICE_COUNT];
};
