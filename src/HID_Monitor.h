#pragma once
#include "../lib/hidapi/hidapi/hidapi.h"
#include <chrono>
#include <cstdint>
#include <functional>
#include <vector>
#include "../lib/amfiprotapi/lib_AmfiProt_API.hpp"
#include "Amfitrack_Devices.h"

#ifdef USE_THREAD_BASED
#include <mutex>
#endif

static constexpr uint16_t VID = 0x0C17;
static constexpr uint16_t PID_Source = 0x0D01;
static constexpr uint16_t PID_Sensor = 0x0D12;
static constexpr size_t USB_REPORT_LENGTH = 64;
static constexpr size_t MAX_NAME_LENGTH_ = 32;
static constexpr uint8_t kUSBReportId = 0x01;

struct HIDMonitorCallbacks
{
	std::function<bool(size_t &queueIdxOut,
					   size_t &lenOut,
					   uint8_t &txIdOut,
					   void *&dataOut)>
		txPoll;

	std::function<void(uint8_t queueIdx)> txDone;

	std::function<void(const uint8_t *data, size_t len)> rxPush;
};

// ─────────────────────────────────────────────────────────────────────────────
class HIDMonitor
{
  public:
	explicit HIDMonitor(HIDMonitorCallbacks callbacks);
	~HIDMonitor();

	bool init();
	void run();
	bool shutdown();

	// Your customer accesses devices through these
	std::vector<AMFITRACK_Sensor> &sensors()
	{
		return _sensors;
	}
	std::vector<AMFITRACK_Source> &sources()
	{
		return _sources;
	}

  private:
	void syncDevices();
	void scanForPid(uint16_t pid);
	void removeDisconnected();

	// Separate probe per type since they have different fields to fill
	bool probeSensorIdentity(AMFITRACK_Sensor &sensor);
	bool probeSourceIdentity(AMFITRACK_Source &source);

	void drainTxQueue();
	void drainRx();

	hid_device *findHandleByTxId(uint8_t txId);

	int hidWrite(hid_device *dev, const void *data, size_t len);
	int hidReadNonBlocking(hid_device *dev, void *data);
	int hidReadTimeout(hid_device *dev, void *data, int timeoutMs);

	HIDMonitorCallbacks _cb;

	bool _initialized = false;
	std::chrono::steady_clock::time_point _lastScanTime{};

	std::vector<AMFITRACK_Sensor> _sensors;
	std::vector<AMFITRACK_Source> _sources;

#ifdef USE_THREAD_BASED
	mutable std::mutex _mutex;
#endif
};