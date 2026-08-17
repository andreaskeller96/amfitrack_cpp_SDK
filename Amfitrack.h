//-----------------------------------------------------------------------------
//
//                              AMFITECH APS
//
//                          ALL RIGHTS RESERVED
//
//-----------------------------------------------------------------------------

#pragma once
#ifdef __cplusplus

//-----------------------------------------------------------------------------
// Section: Includes
//-----------------------------------------------------------------------------
#include "Amfitrack_Sensor.h"
#include "Amfitrack_Source.h"
#include "Amfitrack_config.h"

#include "lib_AmfiProt_API.hpp"

#include <cstdint>
#include <string>
#include <vector>
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
// Section: Class
//-----------------------------------------------------------------------------
class AMFITRACK
{
  public:
	static AMFITRACK &getInstance();

	void init();
	void start_task();
	void stop_task();
	void run();

	bool get_sensor(uint8_t device_id, AMFITRACK_Sensor *sensor) const;
	bool get_source(uint8_t device_id, AMFITRACK_Source *source) const;
	bool get_sensor_by_number(uint8_t device_number, AMFITRACK_Sensor *sensor) const;
	bool get_source_by_number(uint8_t device_number, AMFITRACK_Source *source) const;
	uint8_t get_sensors_active() const;
	uint8_t get_sources_active() const;

	// Silence this USB serial until its application re-enumerates (empty
	// closes): a bootloader that receives any packet stops applying the image.
	void set_firmware_apply_serial(const std::string &usbSerial);

	// Devices in their bootloader; they are in neither the sensor nor source list.
	std::vector<std::string> get_bootloader_serials() const;

	bool setConfiguration(uint8_t DeviceID, uint32_t UID, lib_Generic_Parameter_Value_t parameter);
	bool getConfiguration(uint8_t DeviceID, bool force_all = false);
	ConfigDiscoveryState_t getConfigurationState(uint8_t DeviceID) const;
	//-----------------------------------------------------------------------------
	// Old function will be deprecated
	//-----------------------------------------------------------------------------
  public:
	void initialize_amfitrack();
	void start_amfitrack_task(void);
	void stop_amfitrack_task(void);
	void amfitrack_main_loop(void);

	bool getDeviceActive(uint8_t DeviceID);
	void getDevicePose(uint8_t DeviceID, lib_AmfiProt_Amfitrack_Pose_t *Pose);
	void getDeviceIMU(uint8_t DeviceID, lib_AmfiProt_Amfitrack_IMU_t *imuData);
	void getSensorMeasurements(uint8_t DeviceID, lib_AmfiProt_Amfitrack_Sensor_Measurement_t *SensorMeasurement);
#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
	void getSensorTimestamp(uint8_t DeviceID, std::chrono::steady_clock::time_point *time_stamp);
#endif
	//-----------------------------------------------------------------------------
	//-----------------------------------------------------------------------------

  private:
	AMFITRACK() = default;
	~AMFITRACK() = default;

	AMFITRACK(AMFITRACK const &) = delete;
	AMFITRACK &operator=(AMFITRACK const &) = delete;

	static void background_amfitrack_task(AMFITRACK *);
};

#endif
/** @} */ // end of module
