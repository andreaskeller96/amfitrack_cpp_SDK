///-----------------------------------------------------------------------------
//                              AMFITECH APS
//                          ALL RIGHTS RESERVED
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "usb_connection.h"

#include <map>
#include <tuple>

#ifdef USE_USB
//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------
// #define usb_connection_DEBUG_INFO
#define READ_NONE_BLOCKING
// #define AMFITRACK_DEBUG_READ

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

void libAmfiProt_handle_RespondDeviceID(void *handle, lib_AmfiProt_Frame_t *frame, void *node)
{
    (void)handle;
    AmfitrackNode *pNode = (AmfitrackNode *)node;
    pNode->setTxID(frame->payload[1]);

    uint32_t uuid[3] = {0};
    uint8_t idx = 0;
    uint8_t lshift;
    for (uint8_t i = 2; i < sizeof(lib_AmfiProt_DeviceID_t) - sizeof(uint32_t); ++i)
    {
        if (idx >= 3)
            break;
        lshift = ((i - 2) % 4) * 8;
        uuid[idx] |= frame->payload[i] << lshift;
        if ((i - 2) % 4 == 3)
        {
            idx++;
        }
    }

    pNode->setUUID(uuid);
}

void libAmfiProt_handle_ReplyDeviceName(void *handle, lib_AmfiProt_Frame_t *frame, void *node_name)
{
    (void)handle;
    uint8_t *name = (uint8_t *)node_name;

    size_t str_length = strnlen((char *)(&(frame->payload[1])), MAX_PAYLOAD_SIZE - 1);
    memcpy(node_name, &(frame->payload[1]), str_length);

    if (str_length >= MAX_PAYLOAD_SIZE - 1)
    {
        name[MAX_PAYLOAD_SIZE - 2] = '\0';
    }
    else
    {
        str_length++;
    }
}

//-----------------------------------------------------------------------------
// USB Connection Function Definitions
//-----------------------------------------------------------------------------

usb_connection::usb_connection()
{
    this->_usb_report_id = 0x01;
}

usb_connection::~usb_connection()
{
#ifdef USE_HID
    // shutdown any nodes otherwise we leak memory
    for (const auto &node : _nodes)
    {
        if (hid_device *dev_handle = node->getDeviceHandle())
        {
            hid_close(dev_handle);
            dev_handle = nullptr;
        }
    }
    _nodes.clear();
    hid_close(_DeviceHandle);
    hid_exit();
#else
    libusb_release_interface(_DeviceHandle, 0);
    libusb_close(_DeviceHandle);
    libusb_exit(_ctx);
#endif
}

bool usb_connection::usb_connect_device(uint16_t vid, uint16_t pid)
{
    int err = 0;
#ifdef USE_HID
    // TODO(pkyle): calling hid_init() over and over again is not ideal, right now its essentially
    //  called every 2 seconds, if calling amfitrack_main_loop() repeatedly. Additionally your
    //  assert() is only called when compiled in Debug. This allows initialization to fail in Release.
    err = hid_init();
#else
    err = libusb_init(&_ctx);
#endif
    assert(err == 0);

#ifdef USE_HID
    struct hid_device_info *devs = hid_enumerate(vid, pid);

    if (!devs)
    {
#ifdef usb_connection_DEBUG_INFO
        std::cout << "Could not find any devices matching VID: " << std::hex << vid
                  << " and PID: " << std::hex << pid << std::endl;
#endif
    }
    else
    {
        const hid_device_info *dev = &devs[0];
        do
        {
            bool DeviceAlreadyActive = false;
            for (const auto &node : _nodes)
            {
                const hid_device_info *dev_handleCurrent = hid_get_device_info(node->getDeviceHandle());
                if (dev_handleCurrent && wcscmp(dev_handleCurrent->serial_number, dev->serial_number) == 0)
                {
                    DeviceAlreadyActive = true;
                    break;
                }
            }
            if (DeviceAlreadyActive)
            {
                dev = dev->next;
                continue;
            }
            hid_device *dev_handle = hid_open_path(dev->path);
            if (!dev_handle)
            {
                std::cout << "Unable to open device" << std::endl;
            }
            else
            {
                std::shared_ptr<AmfitrackNode> node(new AmfitrackNode(dev_handle));
                _nodes.push_back(node);
            }
            dev = dev->next;
        } while (dev);
    }

    hid_free_enumeration(devs);
    if (!(_nodes.size()))
    {
        std::cout << "Unable to access any devices with VID: " << std::hex << vid
                  << " and PID: " << std::hex << pid << std::endl;
    }
#else
    _DeviceHandle = libusb_open_device_with_vid_pid(_ctx, _vid, _pid);

    if (libusb_kernel_driver_active(_DeviceHandle, 0) == 1)
    {
        libusb_detach_kernel_driver(_DeviceHandle, 0);
    }
    libusb_claim_interface(_DeviceHandle, 0);
#endif
    return true;
}

bool usb_connection::usb_disconnect_device(uint16_t vid, uint16_t pid)
{
    (void)vid;
    (void)pid;
#ifdef USE_HID
    for (const auto &node : _nodes)
    {
        if (hid_device *dev_handle = node->getDeviceHandle())
        {
            hid_close(dev_handle);
            dev_handle = nullptr;
        }
    }
    _nodes.clear();
#else
    libusb_release_interface(_DeviceHandle, 0);
    libusb_close(_DeviceHandle);
    libusb_exit(_ctx);
#endif
    return 1;
}

int usb_connection::read_blocking(hid_device *dev_handle, void *pData, uint8_t length)
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

#ifdef AMFITRACK_DEBUG_READ
    {
        static std::map<hid_device *, std::tuple<int, int, int, time_t>> stats;
        auto &s = stats[dev_handle];
        auto &calls = std::get<0>(s);
        auto &got_data = std::get<1>(s);
        auto &errors = std::get<2>(s);
        auto &last_print = std::get<3>(s);
        ++calls;
        if (r > 0) ++got_data;
        else if (r < 0) ++errors;
        time_t now = time(0);
        if (last_print == 0) last_print = now;
        if (now - last_print >= 1)
        {
            std::cerr << "[read_blocking dev=" << dev_handle
                      << "] calls=" << calls
                      << " data=" << got_data
                      << " errors=" << errors;
            if (errors > 0)
            {
                const wchar_t *err = hid_error(dev_handle);
                std::wcerr << L" last_err=\"" << (err ? err : L"(null)") << L"\"";
            }
            std::cerr << std::endl;
            calls = got_data = errors = 0;
            last_print = now;
        }
    }
#endif

    memcpy(pData, &(data[2]), USB_REPORT_LENGTH - 2);
    return r - 2;
}

int usb_connection::read_timeout(hid_device *dev_handle, void *pData, uint8_t length, int timeout)
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

int usb_connection::write_blocking(hid_device *dev_handle, void const *pData, uint8_t length)
{
    // Add Report ID as first byte to USB packet
    uint8_t data[USB_REPORT_LENGTH];
    memset(data, 0, USB_REPORT_LENGTH);
    data[0] = _usb_report_id;
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

int usb_connection::set_nonblocking(bool enable)
{
    (void)enable;
    int r = 0;
    hid_device *dev_handle;

    for (auto &node : _nodes)
    {
        dev_handle = node->getDeviceHandle();
        r = hid_set_nonblocking(dev_handle, 1);
    }

    return r;
}

std::vector<std::shared_ptr<AmfitrackNode>> usb_connection::find_nodes()
{
    // int r;
    int actual_length;
    uint8_t packet[USB_REPORT_LENGTH];
    // Setup 'request device id' packet
    lib_AmfiProt_Frame_t tx_frame, rx_frame;
    uint8_t payload;

    AmfiProt_API &_amfiprot_api = AmfiProt_API::getInstance();
    hid_device *dev_handle;

    // For each device in vector of device handles
    for (auto &node : _nodes)
    {
        std::vector<std::shared_ptr<AmfitrackNode>> child_nodes;
        dev_handle = node->getDeviceHandle();
        payload = lib_AmfiProt_PayloadID_RequestDeviceID;
        _amfiprot_api.lib_AmfiProt_EncodeFrame(&tx_frame, &payload, sizeof(payload), libAmfiProt_PayloadType_Common, 0, lib_AmfiProt_destination_Broadcast, lib_AmfiProt_packetType_NoAck);

        uint8_t length = _amfiprot_api.lib_AmfiProt_FrameSize(&tx_frame);
        memset(packet, 0, USB_REPORT_LENGTH);
        memcpy(packet, &tx_frame, length);
        actual_length = this->write_blocking(dev_handle, packet, length);
        if (actual_length < 0)
        {
            std::cout << "Write failed!" << std::endl;
        }

        time_t start_timer, end_timer;
        time(&start_timer);

        do
        {
            actual_length = this->read_timeout(dev_handle, packet, USB_REPORT_LENGTH, 1000);

            if (_amfiprot_api.lib_AmfiProt_DeserializeFrame(&rx_frame, packet, (uint8_t)actual_length))
            {
                if (rx_frame.header.payloadType == libAmfiProt_PayloadType_Common && rx_frame.payload[0] == lib_AmfiProt_PayloadID_ReplyDeviceID)
                {
                    std::shared_ptr<AmfitrackNode> child_node(new AmfitrackNode());
                    libAmfiProt_handle_RespondDeviceID(NULL, &rx_frame, (void *)child_node.get());
                    _amfiprot_api.lib_AmfiProt_ProcessFrame(NULL, &rx_frame, (void *)child_node.get());
                    child_nodes.push_back(child_node);
#ifdef usb_connection_DEBUG_INFO
                    std::cout << *child_node.get() << std::endl;
#endif // usb_connection_DEBUG_INFO
                }
            }
            time(&end_timer);
        } while (difftime(end_timer, start_timer) < 2.0);

        payload = lib_AmfiProt_PayloadID_RequestDeviceName;
        // std::vector<AmfitrackNode *>::iterator iter;
        char name[AmfiProtMaxPayloadLength - 1];
        for (auto &child_node : child_nodes) //(iter = child_nodes.begin(); iter != child_nodes.end(); iter++)
        {
            _amfiprot_api.lib_AmfiProt_EncodeFrame(&tx_frame, &payload, sizeof(payload), libAmfiProt_PayloadType_Common, 0, child_node->getTxID(), lib_AmfiProt_packetType_NoAck);
            length = _amfiprot_api.lib_AmfiProt_FrameSize(&tx_frame);
            memset(packet, 0, USB_REPORT_LENGTH);
            memcpy(packet, &tx_frame, length);

            actual_length = this->write_blocking(dev_handle, packet, length);

            time(&start_timer);
            bool nameReceived = false;
            do
            {
                actual_length = this->read_timeout(dev_handle, packet, USB_REPORT_LENGTH, 1000);

                if (_amfiprot_api.lib_AmfiProt_DeserializeFrame(&rx_frame, packet, (uint8_t)actual_length))
                {
                    if (rx_frame.header.payloadType == libAmfiProt_PayloadType_Common && rx_frame.payload[0] == lib_AmfiProt_PayloadID_ReplyDeviceName)
                    {
                        libAmfiProt_handle_ReplyDeviceName(NULL, &rx_frame, (void *)name);
                        _amfiprot_api.lib_AmfiProt_ProcessFrame(NULL, &rx_frame, (void *)name);
                        nameReceived = true;
                        /* Why does this shut down the program???????? */
                        // child_node->setDevName(name, strnlen((char *)name, sizeof(name)));
#ifdef usb_connection_DEBUG_INFO
                        std::cout << name << std::endl;
#endif // usb_connection_DEBUG_INFO
                    }
                }
                time(&end_timer);
                memset(name, 0, sizeof(name));
            } while ((difftime(end_timer, start_timer) < 2.0) && (!nameReceived)); //(!child_node->getDevName()[0]);
        }
        if (child_nodes.size() == 1)
        {
            child_nodes[0]->setUSBMode(true);
            child_nodes[0]->setDeviceHandle(node->getDeviceHandle());
            node = child_nodes[0];
        }
        else
        {
            node->setHubMode(true);
            for (auto &child_node : child_nodes)
            {
                child_node->setRFMode(true);
                child_node->setHubMode(false);
                node->addChildNode(child_node);
            }
        }
    }

    return _nodes;
}

hid_device *usb_connection::get_device_handle(uint8_t tx_id)
{
    // Find device handle matching tx_id
    // Prioritize direct USB connection
    for (auto node : _nodes)
    {
        if (!(node->isHub()))
        {
            if (tx_id == node->getTxID())
            {
                return node->getDeviceHandle();
            }
        }
    }
    // If tx_id not found amongst direct USB connections, check RF connections
    for (auto node : _nodes)
    {
        if (node->isHub())
        {
            for (auto child_node : node->getChildNodes())
            {
                if (tx_id == child_node->getTxID())
                {
                    return node->getDeviceHandle();
                }
            }
        }
    }

    return NULL;
}

void usb_connection::usb_init(void)
{
    /* Tries to connect to source */
    this->usb_connect_device(VID, PID_Source);
    /* Tries to connect to sensor */
    this->usb_connect_device(VID, PID_Sensor);
    /* Finds all the nodes (Devices connected over a hub) */
    this->find_nodes();
#ifdef READ_NONE_BLOCKING
    /* Sets the USB connections to non-blocking */
    this->set_nonblocking(true);
#endif // READ_NONE_BLOCKING
    time(&this->CheckForDevice_Timer);
}

void usb_connection::usb_run(void)
{
    AmfiProt_API &_amfiprot_api = AmfiProt_API::getInstance();

    time_t CurrentTime;
    time(&CurrentTime);

    if (difftime(CurrentTime, this->CheckForDevice_Timer) >= 2.0)
    {
        // TODO(pkyle): I think it would be a good idea to consider using an event-based workflow to
        //  manage usb connections. Right now, every two seconds, this is being run.
        /* Tries to connect to source */
        this->usb_connect_device(VID, PID_Source);
        /* Tries to connect to sensor */
        this->usb_connect_device(VID, PID_Sensor);
        time(&this->CheckForDevice_Timer);
    }

    size_t QueueIdx = 0;
    size_t QueueDataLength = 0;
    uint8_t tx_id = 0;
    void *TransmitData = NULL;
    if (_amfiprot_api.isDataReadyForTransmit(&QueueIdx, &QueueDataLength, &tx_id, &TransmitData))
    {
        hid_device *dev_handle = this->get_device_handle(tx_id);
        if (dev_handle)
        {
            this->write_blocking(dev_handle, TransmitData, (uint8_t)QueueDataLength);
            _amfiprot_api.set_transmit_ongoing_and_check_respons_request((uint8_t)QueueIdx);
#ifdef USB_CONNECTION_DEBUG_INFO
            std::cout << "Data written" << std::endl;
#endif
        }
        else
        {
            std::cout << "TxID not matching connected devices." << std::endl;
        }
    }

    // Try to receive data
    USB_frame_t rx_frame;
    for (auto node : _nodes)
    {
#ifdef READ_NONE_BLOCKING
        if (this->read_blocking(node->getDeviceHandle(), rx_frame.data, USB_REPORT_LENGTH) >= 0)
#else
        if (this->read_timeout(node->getDeviceHandle(), rx_frame.data, USB_REPORT_LENGTH, 1) >= 0)
#endif
        {
#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
            const auto time_stamp = std::chrono::steady_clock::now();
            _amfiprot_api.deserialize_frame(rx_frame.data, USB_REPORT_LENGTH - 2, time_stamp);
#else
            _amfiprot_api.deserialize_frame(rx_frame.data, USB_REPORT_LENGTH - 2);
#endif
        }
    }
}
#endif
