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

#define AMFITRACK_FW_VERSION_ID 0
#define AMFITRACK_RF_VERSION_ID 1
#define AMFITRACK_HW_VERSION_ID 255
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
	bool is_device_active(uint8_t device_id);

	bool set(uint8_t device_id, bool isActive);
	bool set(uint8_t device_id, char const *name, uint8_t length);
	bool set(uint8_t device_id, uint32_t UUID1, uint32_t UUID2, uint32_t UUID3);
	bool set(uint8_t device_id, FW_t fwVersion);
	bool set(uint8_t device_id, RF_t rfVersion);
	bool set(uint8_t device_id, HW_t hwVersion);
	bool set(uint8_t device_id, uint8_t hubId);
	bool set_hid(uint8_t device_id, hid_device *hidHandle, bool sensor);
	bool set(uint8_t device_id, DeviceConfig_t const &config);

	bool set(uint8_t device_id, Status_t const &status);
	bool set(uint8_t device_id, Pose_t const &pose);
	bool set(uint8_t device_id, IMU_t const &imu);
	bool set(uint8_t device_id, Raw_B_Field_t const &rawBfield);
	bool set(uint8_t device_id, Normalized_B_Field_t const &normBfield);
	bool set(uint8_t device_id, Raw_with_Phase_B_Field_t const &rawWithPhaseBfield);
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
