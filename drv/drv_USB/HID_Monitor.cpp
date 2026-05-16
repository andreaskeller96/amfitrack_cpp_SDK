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
#include "HID_Monitor.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cwchar>
#include "lib_AmfiProt.hpp"
#include "lib_log.h"
#include "Amfitrack.h"
#include "Amfitrack_Devices.h"
#include "Amfitrack_Sensor.h"
#include "Amfitrack_Source.h"

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
static constexpr int kProbeTimeoutMs = 10;
static constexpr int kProbeMaxAttempts = 10;
static constexpr int kScanIntervalS = 1;

//-----------------------------------------------------------------------------
// Section: Function prototypes
//-----------------------------------------------------------------------------
static bool isSameDevice(hid_device *handle, const hid_device_info *info)
{
	if (!handle || !info)
		return false;
	const hid_device_info *cur = hid_get_device_info(handle);
	if (!cur)
		return false;
	if (cur->serial_number && info->serial_number)
		return std::wcscmp(cur->serial_number, info->serial_number) == 0;
	if (cur->path && info->path)
		return std::strcmp(cur->path, info->path) == 0;
	return false;
}

static bool parseIdReply(const lib_AmfiProt_Frame_t &f, uint8_t &deviceId, uint32_t uuid[3])
{
	if (f.header.payloadType != libAmfiProt_PayloadType_Common ||
		f.payload[0] != lib_AmfiProt_PayloadID_ReplyDeviceID)
		return false;

	lib_AmfiProt_DeviceID idFrame;
	memcpy(&idFrame, &f.payload[0], sizeof(lib_AmfiProt_DeviceID));
	deviceId = idFrame.TxID;
	uuid[0] = idFrame.UUID[0];
	uuid[1] = idFrame.UUID[1];
	uuid[2] = idFrame.UUID[2];

	return true;
}

static bool parseNameReply(const lib_AmfiProt_Frame_t &f, char *buf, size_t bufSize)
{
	if (!buf || bufSize == 0)
		return false;
	if (f.header.payloadType != libAmfiProt_PayloadType_Common ||
		f.payload[0] != lib_AmfiProt_PayloadID_ReplyDeviceName)
		return false;
	const size_t len = strnlen(reinterpret_cast<const char *>(&f.payload[1]), bufSize - 1);
	std::memcpy(buf, &f.payload[1], len);
	buf[len] = '\0';
	return true;
}

// Shared probe logic — used by both probeSensorIdentity and probeSourceIdentity
static bool probeDeviceIdentity(hid_device *handle,
								std::function<int(hid_device *, const void *, size_t)> writeFn,
								std::function<int(hid_device *, void *, int)> readFn,
								uint8_t &deviceIdOut,
								uint32_t uuidOut[3],
								char *nameOut,
								size_t nameSize)
{
	AmfiProt_API &api = AmfiProt_API::getInstance();
	lib_AmfiProt_Frame_t txFrame{}, rxFrame{};
	uint8_t packet[USB_REPORT_LENGTH]{};

	// ── Request ID ────────────────────────────────────────────────────────────
	uint8_t payload = lib_AmfiProt_PayloadID_RequestDeviceID;
	api.lib_AmfiProt_EncodeFrame(&txFrame, &payload, sizeof(payload), libAmfiProt_PayloadType_Common, 0, lib_AmfiProt_destination_Broadcast, lib_AmfiProt_packetType_NoAck);
	std::memcpy(packet, &txFrame, api.lib_AmfiProt_FrameSize(&txFrame));
	if (writeFn(handle, packet, api.lib_AmfiProt_FrameSize(&txFrame)) < 0)
		return false;

	bool hasId = false;
	for (int i = 0; i < kProbeMaxAttempts && !hasId; ++i)
	{
		const int n = readFn(handle, packet, kProbeTimeoutMs);
		api.deserialize_frame(packet, (uint8_t)n);
		if (n > 0 && api.lib_AmfiProt_DeserializeFrame(&rxFrame, packet, (uint8_t)n))
			hasId = parseIdReply(rxFrame, deviceIdOut, uuidOut);
	}
	if (!hasId)
		return false;

	// ── Request name ──────────────────────────────────────────────────────────
	payload = lib_AmfiProt_PayloadID_RequestDeviceName;
	api.lib_AmfiProt_EncodeFrame(&txFrame, &payload, sizeof(payload), libAmfiProt_PayloadType_Common, 0, deviceIdOut, lib_AmfiProt_packetType_NoAck);
	std::memset(packet, 0, sizeof(packet));
	std::memcpy(packet, &txFrame, api.lib_AmfiProt_FrameSize(&txFrame));
	if (writeFn(handle, packet, api.lib_AmfiProt_FrameSize(&txFrame)) < 0)
		return false;

	bool hasName = false;
	for (int i = 0; i < kProbeMaxAttempts && !hasName; ++i)
	{
		const int n = readFn(handle, packet, kProbeTimeoutMs);
		api.deserialize_frame(packet, (uint8_t)n);
		if (n > 0 && api.lib_AmfiProt_DeserializeFrame(&rxFrame, packet, (uint8_t)n))
			hasName = parseNameReply(rxFrame, nameOut, nameSize);
	}

	return true; // name is optional — id is enough
}

bool stillPresent(hid_device *handle, uint16_t pid)
{
	if (!handle)
		return false;
	hid_device_info *list = hid_enumerate(VID, pid);
	bool found = false;
	for (const hid_device_info *info = list; info && !found; info = info->next)
		found = isSameDevice(handle, info);
	hid_free_enumeration(list);
	return found;
}
//-----------------------------------------------------------------------------
// Section: Functions
//-----------------------------------------------------------------------------
HIDMonitor::HIDMonitor(HIDMonitorCallbacks callbacks)
	: _cb(std::move(callbacks))
{
}

HIDMonitor::~HIDMonitor()
{
	shutdown();
}

bool HIDMonitor::init()
{
#ifdef USE_THREAD_BASED
	std::lock_guard<std::mutex> lock(_mutex);
#endif
	if (_initialized)
		return true;
	if (hid_init() != 0)
	{
		LOG_E("hid_init() failed");
		return false;
	}
	_initialized = true;
	syncDevices();
	return true;
}

void HIDMonitor::run()
{
#ifdef USE_THREAD_BASED
	std::lock_guard<std::mutex> lock(_mutex);
#endif
	if (!_initialized)
		return;
	syncDevices();
	drainTxQueue();
	drainRx();
}

bool HIDMonitor::shutdown()
{
#ifdef USE_THREAD_BASED
	std::lock_guard<std::mutex> lock(_mutex);
#endif
	for (uint8_t i = 0; i < AMFITRACK_DEVICE_COUNT; i++)
	{
		AMFITRACK_Sensor s;
		AMFITRACK::getInstance().get_sensor(i, &s);
		if (s._dev_handle)
		{
			hid_close(s._dev_handle);
			s._dev_handle = nullptr;
		}
	}
	for (uint8_t i = 0; i < AMFITRACK_DEVICE_COUNT; i++)
	{
		AMFITRACK_Source s;
		AMFITRACK::getInstance().get_source(i, &s);
		if (s._dev_handle)
		{
			hid_close(s._dev_handle);
			s._dev_handle = nullptr;
		}
	}
	if (_initialized)
	{
		hid_exit();
		_initialized = false;
	}
	return true;
}

void HIDMonitor::syncDevices()
{
	const auto now = std::chrono::steady_clock::now();
	if (_lastScanTime != std::chrono::steady_clock::time_point{} &&
		std::chrono::duration_cast<std::chrono::seconds>(now - _lastScanTime).count() < kScanIntervalS)
		return;

	scanForPid(PID_Sensor);
	scanForPid(PID_Source);
	removeDisconnected();
	_lastScanTime = now;
}

void HIDMonitor::scanForPid(uint16_t pid)
{
	hid_device_info *list = hid_enumerate(VID, pid);
	for (const hid_device_info *info = list; info; info = info->next)
	{
		bool alreadyOpen = false;

		if (pid == PID_Sensor)
		{
			for (uint8_t i = 0; i < AMFITRACK_DEVICE_COUNT; i++)
			{
				AMFITRACK_Sensor _sensors;
				AMFITRACK::getInstance().get_sensor(i, &_sensors);
				if (_sensors._dev_handle)
				{
					if (isSameDevice(_sensors._dev_handle, info))
					{
						alreadyOpen = true;
						break;
					}
				}
			}
		}
		else if (pid == PID_Source)
		{
			for (uint8_t i = 0; i < AMFITRACK_DEVICE_COUNT; i++)
			{
				AMFITRACK_Source _sources;
				AMFITRACK::getInstance().get_source(i, &_sources);
				if (isSameDevice(_sources._dev_handle, info))
				{
					alreadyOpen = true;
					break;
				}
			}
		}

		if (alreadyOpen)
			continue;

		hid_device *handle = hid_open_path(info->path);
		if (!handle)
			continue;

		hid_set_nonblocking(handle, 1);

		if (pid == PID_Sensor)
		{
			AMFITRACK_HID sensor;
			sensor._dev_handle = handle;
			if (!probeSensorIdentity(sensor))
			{
				hid_close(handle);
				continue;
			}
			AMFITRACK_Devices::getInstance().set(sensor.deviceId, true);
			AMFITRACK_Devices::getInstance().set_hid(sensor.deviceId, sensor._dev_handle, true);
			LOG_I("Sensor connected on USB: id=%u name=%s", sensor.deviceId, sensor.name);
		}
		else
		{
			AMFITRACK_HID source;
			source._dev_handle = handle;
			if (!probeSourceIdentity(source))
			{
				hid_close(handle);
				continue;
			}
			AMFITRACK_Devices::getInstance().set(source.deviceId, true);
			AMFITRACK_Devices::getInstance().set_hid(source.deviceId, source._dev_handle, false);
			LOG_I("Source connected on USB: id=%u name=%s", source.deviceId, source.name);
		}
	}
	hid_free_enumeration(list);
}

void HIDMonitor::removeDisconnected()
{
	for (uint8_t i = 0; i < AMFITRACK_DEVICE_COUNT; i++)
	{
		bool sensorDisconnected = false;
		bool sourceDisconnected = false;
		AMFITRACK_Sensor sensor;
		AMFITRACK_Source source;
		AMFITRACK_Devices::getInstance().get_sensor(i, &sensor);
		AMFITRACK_Devices::getInstance().get_source(i, &source);

		if (!sensor._dev_handle || stillPresent(sensor._dev_handle, PID_Sensor))
		{
			if (!source._dev_handle || stillPresent(source._dev_handle, PID_Source))
			{
				continue;
			}
			else
			{
				sourceDisconnected = true;
			}
		}
		else
		{
			sensorDisconnected = true;
		}

		if (sensorDisconnected)
		{
			LOG_I("Sensor disconnected on USB: id=%u name=%s", sensor.deviceId, sensor.name);
			if (sensor._dev_handle)
			{
				hid_close(sensor._dev_handle);
				AMFITRACK_Devices::getInstance().set_hid(i, NULL, true);
				sourceDisconnected = false;
			}
		}
		if (sourceDisconnected)
		{
			LOG_I("Source disconnected on USB: id=%u name=%s", source.deviceId, source.name);
			if (source._dev_handle)
			{
				hid_close(source._dev_handle);
				AMFITRACK_Devices::getInstance().set_hid(i, NULL, false);
			}
		}
	}
}

bool HIDMonitor::probeSensorIdentity(AMFITRACK_HID &sensor)
{
	uint8_t deviceId = 0;
	uint32_t uuid[3]{};
	char name[MAX_NAME_LENGTH]{};

	auto writeFn = [this](hid_device *h, const void *d, size_t l)
	{ return hidWrite(h, d, l); };
	auto readFn = [this](hid_device *h, void *d, int t)
	{ return hidReadTimeout(h, d, t); };

	if (!probeDeviceIdentity(sensor._dev_handle, writeFn, readFn, deviceId, uuid, name, 50))
		return false;

	sensor.deviceId = deviceId;
	sensor.uuid[0] = uuid[0];
	sensor.uuid[1] = uuid[1];
	sensor.uuid[2] = uuid[2];
	std::snprintf(sensor.name, sizeof(sensor.name), "%s", name);
	return true;
}

bool HIDMonitor::probeSourceIdentity(AMFITRACK_HID &source)
{
	uint8_t deviceId = 0;
	uint32_t uuid[3]{};
	char name[MAX_NAME_LENGTH]{};

	auto writeFn = [this](hid_device *h, const void *d, size_t l)
	{ return hidWrite(h, d, l); };
	auto readFn = [this](hid_device *h, void *d, int t)
	{ return hidReadTimeout(h, d, t); };

	if (!probeDeviceIdentity(source._dev_handle, writeFn, readFn, deviceId, uuid, name, 50))
		return false;

	source.deviceId = deviceId;
	source.uuid[0] = uuid[0];
	source.uuid[1] = uuid[1];
	source.uuid[2] = uuid[2];
	std::snprintf(source.name, sizeof(source.name), "%s", name);
	return true;
}

void HIDMonitor::drainTxQueue()
{
	if (!_cb.txPoll || !_cb.txDone)
		return;

	size_t dataLen = 0;
	uint8_t txId = 0;
	void *txData = nullptr;

	if (!_cb.txPoll(dataLen, txId, txData))
		return;

	bool sent = false;
	if (txId == 255)
	{
		for (uint8_t i = 0; i < AMFITRACK_DEVICE_COUNT; i++)
		{
			AMFITRACK_Sensor s;
			AMFITRACK_Devices::getInstance().get_sensor(i, &s);
			if (s._dev_handle && hidWrite(s._dev_handle, txData, dataLen) >= 0)
				sent = true;
		}
		for (uint8_t i = 0; i < AMFITRACK_DEVICE_COUNT; i++)
		{
			AMFITRACK_Source s;
			AMFITRACK_Devices::getInstance().get_source(i, &s);
			if (s._dev_handle && hidWrite(s._dev_handle, txData, dataLen) >= 0)
				sent = true;
		}
	}
	else if (hid_device *handle = findHandleByTxId(txId))
	{
		sent = hidWrite(handle, txData, dataLen) >= 0;
	}
	else
	{
		LOG_I("TxID %u not matched to any connected device", txId);
	}

	if (sent)
		_cb.txDone();
}

void HIDMonitor::drainRx()
{
	if (!_cb.rxPush)
		return;

	uint8_t packet[USB_REPORT_LENGTH]{};

	auto readFrom = [&](hid_device *handle)
	{
		while (handle)
		{
			const int n = hidReadNonBlocking(handle, packet);
			if (n <= 0)
				break;

			uint8_t sourceAddress = 0;
			for (uint8_t i = 0; i < AMFITRACK_DEVICE_COUNT; i++)
			{
				AMFITRACK_Sensor sensor;
				AMFITRACK_Source source;
				AMFITRACK_Devices::getInstance().get_sensor(i, &sensor);
				AMFITRACK_Devices::getInstance().get_source(i, &source);

				if (sensor._dev_handle == handle)
				{
					sourceAddress = sensor.deviceId;
					break;
				}
				else if (source._dev_handle == handle)
				{
					sourceAddress = source.deviceId;
					break;
				}
			}
			_cb.rxPush(sourceAddress, packet, (size_t)n);
		}
	};

	for (uint8_t i = 0; i < AMFITRACK_DEVICE_COUNT; i++)
	{
		AMFITRACK_Sensor s;
		AMFITRACK_Devices::getInstance().get_sensor(i, &s);
		if (s._dev_handle)
			readFrom(s._dev_handle);
	}
	for (uint8_t i = 0; i < AMFITRACK_DEVICE_COUNT; i++)
	{
		AMFITRACK_Source s;
		AMFITRACK_Devices::getInstance().get_source(i, &s);
		if (s._dev_handle)
			readFrom(s._dev_handle);
	}
}

hid_device *
HIDMonitor::findHandleByTxId(uint8_t txId)
{
	hid_device *hidHandle = nullptr;
	for (uint8_t i = 0; i < AMFITRACK_DEVICE_COUNT; i++)
	{
		AMFITRACK_Sensor s;
		AMFITRACK_Devices::getInstance().get_sensor(i, &s);
		if (s.deviceId == txId)
		{

			if (!s._dev_handle)
			{
				AMFITRACK_Sensor hub;
				AMFITRACK_Devices::getInstance().get_sensor(s.hub_ID, &hub);
				if (hub._dev_handle)
				{
					hidHandle = hub._dev_handle;
				}
			}
			else
			{
				hidHandle = s._dev_handle;
			}
			return hidHandle;
		}
	}
	for (uint8_t i = 0; i < AMFITRACK_DEVICE_COUNT; i++)
	{
		AMFITRACK_Source s;
		AMFITRACK_Devices::getInstance().get_source(i, &s);
		if (s.deviceId == txId)
			return s._dev_handle;
	}

	return nullptr;
}

int HIDMonitor::hidWrite(hid_device *dev, const void *data, size_t len)
{
	if (!dev || !data)
		return -1;
	uint8_t buf[USB_REPORT_LENGTH]{};
	buf[0] = kUSBReportId;
	std::memcpy(buf + 1, data, std::min(len, USB_REPORT_LENGTH - 1));
	return hid_write(dev, buf, USB_REPORT_LENGTH);
}

int HIDMonitor::hidReadNonBlocking(hid_device *dev, void *data)
{
	if (!dev || !data)
		return -1;
	uint8_t raw[USB_REPORT_LENGTH]{};
	const int n = hid_read(dev, raw, USB_REPORT_LENGTH);
	if (n <= 2)
		return n;
	std::memcpy(data, raw + 2, n - 2);
	return n - 2;
}

int HIDMonitor::hidReadTimeout(hid_device *dev, void *data, int timeoutMs)
{
	if (!dev || !data)
		return -1;
	uint8_t raw[USB_REPORT_LENGTH]{};
	const int n = hid_read_timeout(dev, raw, USB_REPORT_LENGTH, timeoutMs);
	if (n <= 2)
		return n;
	std::memcpy(data, raw + 2, n - 2);
	return n - 2;
}
