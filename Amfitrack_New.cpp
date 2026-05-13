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
#include "Amfitrack_New.h"

#include "src/project_conf.h"
#include "src/HID_Monitor.h"

#include "lib/amfiprotapi/lib_AmfiProt_API.hpp"

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
static AmfiProt_API *amfiprot_api = nullptr;
static std::unique_ptr<HIDMonitor> hid_monitor = nullptr;
volatile static bool stop_running = false;
//-----------------------------------------------------------------------------
// Section: Function prototypes
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Section: Functions
//-----------------------------------------------------------------------------
static void _run_all_amfitrack()
{
#ifdef USE_USB
	hid_monitor->run();
#endif
	amfiprot_api->amfiprot_run();
}

AMFITRACK_NEW &AMFITRACK_NEW::getInstance()
{
	static AMFITRACK_NEW instance;
	return instance;
}

AMFITRACK_NEW::AMFITRACK_NEW()
{
	setup_device_slots();
}

void AMFITRACK_NEW::background_amfitrack_task(AMFITRACK_NEW *inst)
{
	while (!stop_running)
	{
		_run_all_amfitrack();

		std::this_thread::sleep_for(std::chrono::milliseconds(1));
	}
}

void AMFITRACK_NEW::init()
{
#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	amfiprot_api = &AmfiProt_API::getInstance();
#ifdef USE_USB
	if (!hid_monitor)
	{
		HIDMonitorCallbacks cb;

		cb.txPoll = [](size_t &queueIdx, size_t &len, uint8_t &txId, void *&data) -> bool
		{
			return amfiprot_api->isDataReadyForTransmit(&queueIdx, &len, &txId, &data);
		};

		cb.txDone = [](uint8_t queueIdx)
		{
			amfiprot_api->set_transmit_ongoing_and_check_respons_request(queueIdx);
		};

		cb.rxPush = [](const uint8_t *data, size_t len)
		{
			amfiprot_api->deserialize_frame(data, static_cast<uint8_t>(len));
		};

		hid_monitor = std::make_unique<HIDMonitor>(std::move(cb));
		hid_monitor->init();
	}
#endif

	setup_device_slots();
}

void AMFITRACK_NEW::start_task()
{
	stop_running = false;
	// Create a thread object
	std::thread background_thread(background_amfitrack_task, this);

	background_thread.detach();
}

void AMFITRACK_NEW::stop_task()
{
	stop_running = true;
}

void AMFITRACK_NEW::run()
{
	_run_all_amfitrack();
}

void AMFITRACK_NEW::reset_devices()
{
#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	setup_device_slots();
}

bool AMFITRACK_NEW::set(uint8_t device_id, Pose_t const &pose)
{
	if (!is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	_sensors[device_id].pose = pose;
	_sensors[device_id].lastTimeSeenMs = get_time_ms();
	_sensors[device_id].active = true;
	return true;
}

bool AMFITRACK_NEW::set(uint8_t device_id, IMU_t const &imu)
{
	if (!is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	_sensors[device_id].imu = imu;
	_sensors[device_id].lastTimeSeenMs = get_time_ms();
	_sensors[device_id].active = true;
	return true;
}

#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
bool AMFITRACK_NEW::set(uint8_t device_id, std::chrono::steady_clock::time_point timestamp)
{
	if (!is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	_sensors[device_id].sensorTimestamp = timestamp;
	_sensors[device_id].lastTimeSeenMs = get_time_ms();
	_sensors[device_id].active = true;
	return true;
}
#endif

bool AMFITRACK_NEW::set(uint8_t device_id, Current_t const &current)
{
	if (!is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	_sources[device_id].current = current;
	_sources[device_id].lastTimeSeenMs = get_time_ms();
	_sources[device_id].active = true;
	return true;
}

bool AMFITRACK_NEW::set(uint8_t device_id, Frequency_t const &frequency)
{
	if (!is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	_sources[device_id].frequency = frequency;
	_sources[device_id].lastTimeSeenMs = get_time_ms();
	_sources[device_id].active = true;
	return true;
}

bool AMFITRACK_NEW::set(uint8_t device_id, Voltage_t const &voltage)
{
	if (!is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	_sources[device_id].voltage = voltage;
	_sources[device_id].lastTimeSeenMs = get_time_ms();
	_sources[device_id].active = true;
	return true;
}

bool AMFITRACK_NEW::set(uint8_t device_id, Calibration_t const &calibration)
{
	if (!is_valid_device_id(device_id))
	{
		return false;
	}

#ifdef USE_THREAD_BASED
	const std::lock_guard<std::mutex> lock(_mutex);
#endif

	_sources[device_id].calibration = calibration;
	_sources[device_id].lastTimeSeenMs = get_time_ms();
	_sources[device_id].active = true;
	return true;
}

bool AMFITRACK_NEW::get_sensor(uint8_t device_id, AMFITRACK_Sensor *sensor) const
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

bool AMFITRACK_NEW::reset_sensor(uint8_t device_id)
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

bool AMFITRACK_NEW::get_source(uint8_t device_id, AMFITRACK_Source *source) const
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

bool AMFITRACK_NEW::reset_source(uint8_t device_id)
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

bool AMFITRACK_NEW::is_valid_device_id(uint8_t device_id)
{
	return (device_id < AMFITRACK_NEW_DEVICE_COUNT) &&
		   (device_id != AMFITRACK_NEW_BROADCAST_DEVICE_ID);
}

std::size_t AMFITRACK_NEW::device_count()
{
	return AMFITRACK_NEW_DEVICE_COUNT;
}

uint32_t AMFITRACK_NEW::get_time_ms()
{
	return static_cast<uint32_t>(
		std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now().time_since_epoch())
			.count());
}

void AMFITRACK_NEW::setup_device_slots()
{
	for (std::size_t device_id = 0; device_id < AMFITRACK_NEW_DEVICE_COUNT; device_id++)
	{
		_sensors[device_id].reset();
		_sensors[device_id].deviceId = static_cast<uint8_t>(device_id);

		_sources[device_id].reset();
		_sources[device_id].deviceId = static_cast<uint8_t>(device_id);
	}
}
