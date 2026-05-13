#include "project_conf.h"
#include "src/project_conf.h"
#include "lib_AmfiProt_API.hpp"
#include "Amfitrack.hpp"
#include "lib/lib_log/lib_log.h"

#ifdef USE_USB
#include "HID_Monitor.h"
#endif
#ifdef USE_THREAD_BASED
#include <iostream>
#include <thread>
#endif

// #define AMFITRACK_DEBUG_INFO

static bool stop_running = false;

AMFITRACK::AMFITRACK()
{
	// Initialize Name with null characters
	for (int i = 0; i < MAX_NUMBER_OF_DEVICES; i++)
	{
		memset(Name[i], 0, MAX_NAME_LENGTH);
		DeviceActive[i] = false;
		Position[i].position_x_in_m = 0;
		Position[i].position_y_in_m = 0;
		Position[i].position_z_in_m = 0;

		Position[i].orientation_x = 0;
		Position[i].orientation_y = 0;
		Position[i].orientation_z = 0;
		Position[i].orientation_w = 0;
	}
}

AMFITRACK::~AMFITRACK()
{
}

void AMFITRACK::background_amfitrack_task(AMFITRACK *inst)
{
	(void)inst;
#if defined(USE_THREAD_BASED)
#ifdef USE_USB
	AmfiProt_API &api = AmfiProt_API::getInstance();
	HIDMonitorCallbacks cb;

	cb.txPoll = [&api](size_t &queueIdx, size_t &len, uint8_t &txId, void *&data) -> bool
	{
		return api.isDataReadyForTransmit(&queueIdx, &len, &txId, &data);
	};

	cb.txDone = [&api](uint8_t queueIdx)
	{
		api.set_transmit_ongoing_and_check_respons_request(queueIdx);
	};

	cb.rxPush = [&api](const uint8_t *data, size_t len)
	{
		// #if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
		//     api.deserialize_frame(data, (uint8_t)len, std::chrono::steady_clock::now());
		// #else
		api.deserialize_frame(data, (uint8_t)len);
		// #endif
	};

	HIDMonitor monitor(std::move(cb));
	monitor.init();
#endif
	AmfiProt_API &amfiprot_api = AmfiProt_API::getInstance();
	AMFITRACK &AMFITRACK = AMFITRACK::getInstance();

	LOG_D("Background thread started!");

	while (!stop_running)
	{
#ifdef USE_USB
		monitor.run();
#endif

		amfiprot_api.amfiprot_run();

		for (uint8_t devices = 0; devices < MAX_NUMBER_OF_DEVICES; devices++)
		{
			if (AMFITRACK.getDeviceActive(devices))
			{
				AMFITRACK.checkDeviceDisconnected(devices);
			}
		}

		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
#endif
}

void AMFITRACK::start_amfitrack_task(void)
{
#if defined(USE_THREAD_BASED)
	stop_running = false;

	LOG_D("Starting Background thread!");

	// Create a thread object
	std::thread background_thread(background_amfitrack_task, this);

	background_thread.detach();
#endif
}

void AMFITRACK::stop_amfitrack_task(void)
{
	stop_running = true;
}

void AMFITRACK::amfitrack_main_loop(void)
{
#ifdef USE_USB
	AmfiProt_API &api = AmfiProt_API::getInstance();
	HIDMonitorCallbacks cb;

	cb.txPoll = [&api](size_t &queueIdx, size_t &len, uint8_t &txId, void *&data) -> bool
	{
		return api.isDataReadyForTransmit(&queueIdx, &len, &txId, &data);
	};

	cb.txDone = [&api](uint8_t queueIdx)
	{
		api.set_transmit_ongoing_and_check_respons_request(queueIdx);
	};

	cb.rxPush = [&api](const uint8_t *data, size_t len)
	{
		// #if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
		//     api.deserialize_frame(data, (uint8_t)len, std::chrono::steady_clock::now());
		// #else
		api.deserialize_frame(data, (uint8_t)len);
		// #endif
	};

	HIDMonitor monitor(std::move(cb));
	monitor.init();
#endif
	AmfiProt_API &amfiprot_api = AmfiProt_API::getInstance();
	AMFITRACK &AMFITRACK = AMFITRACK::getInstance();

#ifdef USE_USB
	monitor.run();
#endif
	amfiprot_api.amfiprot_run();

#ifdef USE_ACTIVE_DEVICE_HANDLING
	for (uint8_t devices = 0; devices < MAX_NUMBER_OF_DEVICES; devices++)
	{
		if (AMFITRACK.getDeviceActive(devices))
		{
			AMFITRACK.checkDeviceDisconnected(devices);
		}
	}
#endif
}

void AMFITRACK::initialize_amfitrack()
{
#ifdef USE_USB
	AmfiProt_API &api = AmfiProt_API::getInstance();
	HIDMonitorCallbacks cb;

	cb.txPoll = [&api](size_t &queueIdx, size_t &len, uint8_t &txId, void *&data) -> bool
	{
		return api.isDataReadyForTransmit(&queueIdx, &len, &txId, &data);
	};

	cb.txDone = [&api](uint8_t queueIdx)
	{
		api.set_transmit_ongoing_and_check_respons_request(queueIdx);
	};

	cb.rxPush = [&api](const uint8_t *data, size_t len)
	{
		api.deserialize_frame(data, (uint8_t)len);
	};

	HIDMonitor monitor(std::move(cb));
	monitor.init();
#endif
}

void AMFITRACK::setDeviceName(uint8_t DeviceID, char *name, uint8_t length)
{
	// Check for valid device ID and name length
	if (length >= MAX_NAME_LENGTH)
		return;
#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(mutName);
#endif // USE_THREAD_BASED
	for (uint8_t i = 0; i < length; i++)
	{
		Name[DeviceID][i] = name[i];
	}
	Name[DeviceID][length] = '\0'; // Ensure null termination

	LOG_D(Name[DeviceID]);
}

void AMFITRACK::setConfiguration(uint8_t DeviceID, uint32_t UID, lib_Generic_Parameter_Value_t parameter)
{
	AmfiProt_API &amfiprot_api = AmfiProt_API::getInstance();
	lib_AmfiProt_ConfigValueUID_t ConfigurationPayload = {};
	ConfigurationPayload.payloadID = lib_AmfiProt_PayloadID_SetConfigurationValueUID;
	ConfigurationPayload.uid = UID;
	ConfigurationPayload.value = parameter;
	uint8_t payloadSize = sizeof(ConfigurationPayload) - sizeof(ConfigurationPayload.value) + lib_Generic_Parameter_SizeWithType(ConfigurationPayload.value);

	amfiprot_api.queue_frame(&ConfigurationPayload, payloadSize, libAmfiProt_PayloadType_Common, lib_AmfiProt_packetType_NoAck, DeviceID);
}

// This function checks if the device is disconnected,
// sets the DeviceActive variable if it has
// and returns true if it has been disconnected, false if it is still active.
bool AMFITRACK::checkDeviceDisconnected(uint8_t DeviceID)
{
#ifdef USE_ACTIVE_DEVICE_HANDLING
	time_t CurrentTime = time(0);

	if (difftime(CurrentTime, DeviceLastTimeSeen[DeviceID]) > 5.0)
	{
#ifdef USE_THREAD_BASED
		const std::lock_guard<std::mutex> lock(mutDeviceActive);
#endif // USE_THREAD_BASED
		DeviceActive[DeviceID] = false;
		LOG_I("Device %u disconnected", DeviceID);
		return true;
	}
#endif
	return false;
}

void AMFITRACK::setDeviceActive(uint8_t DeviceID)
{
#ifdef USE_ACTIVE_DEVICE_HANDLING
#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(mutDeviceActive);
#endif // USE_THREAD_BASED
	if (!DeviceActive[DeviceID])
		LOG_I("Device %u connected", DeviceID);
	DeviceActive[DeviceID] = true;
	DeviceLastTimeSeen[DeviceID] = time(0);
	LOG_D("Device is active", DeviceID);

#endif
}

bool AMFITRACK::getDeviceActive(uint8_t DeviceID)
{
#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(mutDeviceActive);
#endif // USE_THREAD_BASED
	return DeviceActive[DeviceID];
}

void AMFITRACK::setDevicePose(uint8_t DeviceID, lib_AmfiProt_Amfitrack_Pose_t Pose)
{
#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(mutPosition);
#endif // USE_THREAD_BASED
	memcpy(&Position[DeviceID], &Pose, sizeof(lib_AmfiProt_Amfitrack_Pose_t));
}

void AMFITRACK::getDevicePose(uint8_t DeviceID, lib_AmfiProt_Amfitrack_Pose_t *Pose)
{
	if (!getDeviceActive(DeviceID))
		return;
#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(mutPosition);
#endif // USE_THREAD_BASED
	memcpy(Pose, &Position[DeviceID], sizeof(lib_AmfiProt_Amfitrack_Pose_t));
}

void AMFITRACK::setDeviceIMU(uint8_t DeviceID, lib_AmfiProt_Amfitrack_IMU_t imuData)
{
#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(mutIMU);
#endif // USE_THREAD_BASED
	memcpy(&IMUData[DeviceID], &imuData, sizeof(lib_AmfiProt_Amfitrack_IMU_t));
}

void AMFITRACK::getDeviceIMU(uint8_t DeviceID, lib_AmfiProt_Amfitrack_IMU_t *imuData)
{
	if (!getDeviceActive(DeviceID))
		return;
#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(mutSensorMeasurements);
#endif // USE_THREAD_BASED
	memcpy(imuData, &IMUData[DeviceID], sizeof(lib_AmfiProt_Amfitrack_IMU_t));
}

void AMFITRACK::setSensorMeasurements(uint8_t DeviceID, lib_AmfiProt_Amfitrack_Sensor_Measurement_t SensorMeasurement)
{
#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(mutSensorMeasurements);
#endif // USE_THREAD_BASED
	memcpy(&SensorMeasurements[DeviceID], &SensorMeasurement, sizeof(lib_AmfiProt_Amfitrack_Sensor_Measurement_t));
}

void AMFITRACK::getSensorMeasurements(uint8_t DeviceID, lib_AmfiProt_Amfitrack_Sensor_Measurement_t *SensorMeasurement)
{
	if (!getDeviceActive(DeviceID))
		return;
#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(mutSensorMeasurements);
#endif // USE_THREAD_BASED
	memcpy(SensorMeasurement, &SensorMeasurements[DeviceID], sizeof(lib_AmfiProt_Amfitrack_Sensor_Measurement_t));
}

#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
std::chrono::steady_clock::time_point getTimestampMicroseconds()
{
	std::chrono::steady_clock::time_point time;
#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
	time = std::chrono::steady_clock::now();
#else
	// Unknown OS
#endif
	return time;
}

void AMFITRACK::setSensorTimestamp(uint8_t DeviceID, std::chrono::steady_clock::time_point time_stamp)
{
#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(mutSensorTimestamps);
#endif // USE_THREAD_BASED
	memcpy(&SensorTimestamps[DeviceID], &time_stamp, sizeof(std::chrono::steady_clock::time_point));
}

void AMFITRACK::getSensorTimestamp(uint8_t DeviceID, std::chrono::steady_clock::time_point *time_stamp)
{
	if (!getDeviceActive(DeviceID))
		return;
#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(mutSensorTimestamps);
#endif // USE_THREAD_BASED
	memcpy(time_stamp, &SensorTimestamps[DeviceID], sizeof(std::chrono::steady_clock::time_point));
}

#endif
