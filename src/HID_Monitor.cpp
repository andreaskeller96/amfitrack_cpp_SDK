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
#include "lib_AmfiProt_API.hpp"
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
// Section: Function prototypes
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Section: Functions
//-----------------------------------------------------------------------------
HIDMonitor::HIDMonitor()
{
}

HIDMonitor::~HIDMonitor()
{
    shutdown();
}

bool HIDMonitor::init()
{
    const std::lock_guard<std::mutex> lock(mutex);
    if (_initialized)
    {
        return true;
    }

    const int error = hid_init();
    if (error != 0)
    {
        printf("HID initialization failed \r\n");
        return false;
    }

    _initialized = true;
    return true;
}
void HIDMonitor::run()
{
    scanMatchingDevices();

    AmfiProt_API &_amfiprot_api = AmfiProt_API::getInstance();
}

bool HIDMonitor::shutdown()
{
    std::lock_guard<std::mutex> lock(mutex);

    if (_initialized)
    {
        hid_exit();
        _initialized = false;
    }
    return true;
}

void HIDMonitor::scanMatchingDevices()
{

    struct hid_device_info *sensor_devs = hid_enumerate(VID, PID_Sensor);
    struct hid_device_info *source_devs = hid_enumerate(VID, PID_Source);

    if (sensor_devs)
    {
        const hid_device_info *dev = &sensor_devs[0];
        do
        {
            hid_device *dev_handle = hid_open_path(dev->path);

            dev = dev->next;
        } while (dev);
    }

    if (source_devs)
    {
        const hid_device_info *dev = &source_devs[0];
        do
        {
            hid_device *dev_handle = hid_open_path(dev->path);

            dev = dev->next;
        } while (dev);
    }
}

int HIDMonitor::read_blocking(hid_device *dev_handle, void *pData, uint8_t length)
{
    (void)length;
    uint8_t data[USB_REPORT_LENGTH];

    int r = 0;
    memset(data, 0, USB_REPORT_LENGTH);
#ifdef USE_HID
    r = hid_read(dev_handle, data, USB_REPORT_LENGTH);
#else
    int actual_length = 0;
    r = libusb_bulk_transfer(_DeviceHandle, LIBUSB_ENDPOINT_IN, data, USB_REPORT_LENGTH, &actual_length, 0);
#endif
    memcpy(pData, &(data[2]), USB_REPORT_LENGTH - 2);
    return r - 2;
}

int HIDMonitor::read_timeout(hid_device *dev_handle, void *pData, uint8_t length, int timeout)
{
    (void)length;
    uint8_t data[USB_REPORT_LENGTH];

    int r = 0;
    memset(data, 0, USB_REPORT_LENGTH);
#ifdef USE_HID
    r = hid_read_timeout(dev_handle, data, USB_REPORT_LENGTH, timeout);
#else
    int actual_length = 0;
    r = libusb_bulk_transfer(_DeviceHandle, LIBUSB_ENDPOINT_IN, data, USB_REPORT_LENGTH, &actual_length, 0);
#endif
    memcpy(pData, &(data[2]), USB_REPORT_LENGTH - 2);
    return r - 2;
}

int HIDMonitor::write_blocking(hid_device *dev_handle, void const *pData, uint8_t length)
{
    // Add Report ID as first byte to USB packet
    uint8_t data[USB_REPORT_LENGTH];
    memset(data, 0, USB_REPORT_LENGTH);
    data[0] = kUSBReportId;
    memcpy(&(data[1]), pData, length);

    // Transfer USB packet
    int r = 0;
#ifdef USE_HID
    r = hid_write(dev_handle, data, USB_REPORT_LENGTH);
#else
    int actual_length = 0;
    r = libusb_bulk_transfer(_DeviceHandle, LIBUSB_ENDPOINT_OUT, data, USB_REPORT_LENGTH, &actual_length, 0);
#endif
    return r;
}

int HIDMonitor::set_nonblocking(hid_device *dev_handle, bool enable)
{
    (void)enable;
    int r = 0;
    r = hid_set_nonblocking(dev_handle, 1);

    return r;
}