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
#include "project_conf.h"

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#ifdef USE_THREAD_BASED
#include <mutex>
#endif // USE_THREAD_BASED

#include "hidapi.h"



//-----------------------------------------------------------------------------
// Section: Define
//-----------------------------------------------------------------------------
#define USB_REPORT_LENGTH 64
#define USE_HID
//-----------------------------------------------------------------------------
// Section: Typedef
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Section: Macro
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Section: Variables
//-----------------------------------------------------------------------------
const uint32_t VID = 0x0C17;
const uint32_t PID_Source = 0x0D01; // Source
const uint32_t PID_Sensor = 0x0D12; // Sensor

//-----------------------------------------------------------------------------
// Section: Function prototypes
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Section: Classes
//-----------------------------------------------------------------------------

class HIDMonitor
{
    static constexpr uint8_t kUSBReportId = 0x01;

public:
    static HIDMonitor &getInstance()
    {
        static HIDMonitor instance;
        return instance;
    }

    bool init();
    void run();
    bool shutdown();

private:
    HIDMonitor();
    ~HIDMonitor();

    int read_blocking(hid_device *dev_handle, void *pData, uint8_t length);
    int read_timeout(hid_device *dev_handle, void *pData, uint8_t length, int timeout);
    int write_blocking(hid_device *dev_handle, void const *pData, uint8_t length);
    int set_nonblocking(hid_device *dev_handle, bool enable);

    void scanMatchingDevices();

    bool _initialized;
#ifdef USE_THREAD_BASED
    std::mutex mutex;
#endif
};