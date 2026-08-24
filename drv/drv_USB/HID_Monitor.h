//-----------------------------------------------------------------------------
//
//                              AMFITECH APS
//
//                          ALL RIGHTS RESERVED
//
//-----------------------------------------------------------------------------
#pragma once
//-----------------------------------------------------------------------------
// Section: Includes
//-----------------------------------------------------------------------------
#include "hidapi.h"
#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>
#include "lib_AmfiProt_API.hpp"
#include "Amfitrack_Sensor.h"
#include "Amfitrack_Source.h"

#ifdef USE_THREAD_BASED
#include <mutex>
#endif

//-----------------------------------------------------------------------------
// Section: Define
//-----------------------------------------------------------------------------
static constexpr uint16_t VID = 0x0C17;
static constexpr uint16_t PID_Source = 0x0D01;
static constexpr uint16_t PID_Sensor = 0x0D12;
static constexpr size_t USB_REPORT_LENGTH = 64;
static constexpr size_t MAX_NAME_LENGTH_ = 32;
static constexpr uint8_t kUSBReportId = 0x01;

// Safety net only: a bootloader must never be probed in the first place.
static constexpr const char *kBootloaderName = "Medability Bootloader";

// Fast enough that neither reboot of an updating device falls between scans.
static constexpr uint32_t kApplyScanIntervalMs = 100;

// Drop an unwritable frame after this many passes so the queue keeps draining.
static constexpr int kMaxTxAttempts = 3;

// An updating device re-enumerates twice: bootloader, then new application.
static constexpr int kBootloaderAppearance = 1;
static constexpr int kApplicationAppearance = 2;

//-----------------------------------------------------------------------------
// Section: Typedef
//-----------------------------------------------------------------------------
enum class ProbeResult
{
	NoReply,	// no usable reply, or not the device type that was asked for
	Bootloader, // replied, but the device is sitting in its bootloader
	Identified, // replied as the expected device type
};

//-----------------------------------------------------------------------------
// Section: Typedef
//-----------------------------------------------------------------------------
struct HIDMonitorCallbacks
{
	std::function<bool(size_t &lenOut,
					   uint8_t &txIdOut,
					   void *&dataOut)>
		txPoll;

	std::function<void(bool removeFromQueue)> txDone;

	std::function<void(uint8_t sourceAddres, const uint8_t *data, size_t len)> rxPush;
};
//-----------------------------------------------------------------------------
// Section: Macro
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Section: Variables
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Section: Class
//-----------------------------------------------------------------------------
class AMFITRACK_HID
{
  public:
	uint8_t deviceId;
	char name[MAX_NAME_LENGTH];
	uint32_t uuid[3];

	hid_device *_dev_handle;
};

class HIDMonitor
{
  public:
	explicit HIDMonitor(HIDMonitorCallbacks callbacks);
	~HIDMonitor();

	bool init();
	void run();
	bool shutdown();

	void set_hid_device(uint8_t deviceID, hid_device *handle);

	// Silence this USB serial until its application re-enumerates (empty
	// closes): a bootloader that receives any packet stops applying the image.
	void set_firmware_apply(const std::string &usbSerial);

	// Devices in their bootloader; they are in neither the sensor nor source list.
	std::vector<std::string> bootloader_serials() const;

  private:
	void syncDevices();
	void scanForPid(uint16_t pid);
	void removeDisconnected();

	// Separate probe per type since they have different fields to fill
	ProbeResult probeSensorIdentity(AMFITRACK_HID &sensor);
	ProbeResult probeSourceIdentity(AMFITRACK_HID &source);

	void drainTxQueue();
	void drainRx();

	hid_device *findHandleByTxId(uint8_t txId);

	int hidWrite(hid_device *dev, const void *data, size_t len);
	int hidReadNonBlocking(hid_device *dev, void *data);
	int hidReadTimeout(hid_device *dev, void *data, int timeoutMs);

	// Follow the updating device on and off the bus without touching it.
	void updateApplyState();
	bool applySuppressed(const hid_device_info *info) const;

	HIDMonitorCallbacks _cb;

	bool _initialized = false;
	uint32_t _lastScanTime = 0;

	// The "applying firmware" window (see set_firmware_apply).
	std::string _applySerial;    // device being updated; empty = window closed
	bool _applyPresent = false;  // was it on the bus at the previous scan
	int _applyAppearances = 0;   // absent->present transitions since it opened

	// A frame nothing can write must not pin the head of the outgoing queue.
	int _txFailedAttempts = 0;

#ifdef USE_THREAD_BASED
	mutable std::mutex _mutex;
#endif
};
