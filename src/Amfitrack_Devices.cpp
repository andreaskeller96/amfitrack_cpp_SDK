#include "Amfitrack_Devices.h"

#include <cstring>

AMFITRACK_Sensor::AMFITRACK_Sensor()
	: AMFITRACK_Sensor(0)
{
}

AMFITRACK_Sensor::AMFITRACK_Sensor(uint8_t id)
{
	reset();
	deviceId = id;
}

void AMFITRACK_Sensor::reset()
{
	deviceId = 0;
	std::memset(name, 0, sizeof(name));
	std::memset(uuid, 0, sizeof(uuid));
	std::memset(&FW_Version, 0, sizeof(FW_Version));
	std::memset(&RF_Version, 0, sizeof(RF_Version));
	std::memset(&HW_Version, 0, sizeof(HW_Version));
	active = false;
	calcId = 0;
	_dev_handle = nullptr;
	std::memset(&pose, 0, sizeof(pose));
	std::memset(&imu, 0, sizeof(imu));
#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
	sensorTimestamp = std::chrono::steady_clock::time_point{};
#endif
	lastTimeSeenMs = 0;
}

AMFITRACK_Source::AMFITRACK_Source()
	: AMFITRACK_Source(0)
{
}

AMFITRACK_Source::AMFITRACK_Source(uint8_t id)
{
	reset();
	deviceId = id;
}

void AMFITRACK_Source::reset()
{
	deviceId = 0;
	std::memset(name, 0, sizeof(name));
	std::memset(uuid, 0, sizeof(uuid));
	std::memset(&FW_Version, 0, sizeof(FW_Version));
	std::memset(&RF_Version, 0, sizeof(RF_Version));
	std::memset(&HW_Version, 0, sizeof(HW_Version));
	active = false;
	_dev_handle = nullptr;
	std::memset(&current, 0, sizeof(current));
	std::memset(&frequency, 0, sizeof(frequency));
	std::memset(&voltage, 0, sizeof(voltage));
	std::memset(&calibration, 0, sizeof(calibration));
	lastTimeSeenMs = 0;
}
