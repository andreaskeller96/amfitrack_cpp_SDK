#include "lib/amfiprotapi/lib_AmfiProt_API.hpp"
#include "Amfitrack.hpp"
#ifdef USE_USB
#include "src/usb_connection.h"
#endif
#ifdef USE_THREAD_BASED
#include <iostream>
#include <thread>
#include <iomanip>
#endif
// #define AMFITRACK_DEBUG_INFO

using namespace AMFITRACK_API;

static bool stop_running = false;

AMFITRACK::AMFITRACK()
{
    // Initialize Name with null characters
    for (int i = 0; i < MAX_NUMBER_OF_DEVICES; i++)
    {
        memset(Name[i], 0, MAX_NAME_LENGTH);
        DeviceActive[i] = false;
        DeviceTypes[i] = DeviceType::Unknown;
        Position[i].position_x_in_m = 0;
        Position[i].position_y_in_m = 0;
        Position[i].position_z_in_m = 0;

        Position[i].orientation_x = 0;
        Position[i].orientation_y = 0;
        Position[i].orientation_z = 0;
        Position[i].orientation_w = 0;

        ConfigEnumStates[i].status = ConfigEnumStatus::Idle;
        ConfigEnumStates[i].waiting_for = ConfigWaitingFor::Nothing;
        ConfigEnumStates[i].expected_count = 0;
        ConfigEnumStates[i].pending_index = 0;
        ConfigEnumStates[i].pending_uid = 0;
        ConfigEnumStates[i].retry_count = 0;
    }
}

AMFITRACK::~AMFITRACK()
{
}

void AMFITRACK::background_amfitrack_task(AMFITRACK *inst)
{
    (void)inst;
#if defined(USE_THREAD_BASED)
    /* Creates instance of USB */
#ifdef USE_USB
    usb_connection &usb = usb_connection::getInstance();
#endif
    AmfiProt_API &amfiprot_api = AmfiProt_API::getInstance();
    AMFITRACK &AMFITRACK = AMFITRACK::getInstance();

#ifdef AMFITRACK_DEBUG_INFO
    std::cout << "Background thread started!" << std::endl;
#endif // AMFITRACK_DEBUG_INFO

    while (!stop_running)
    {
#ifdef USE_USB
        usb.usb_run();
#endif

        amfiprot_api.amfiprot_run();

        for (uint8_t devices = 0; devices < MAX_NUMBER_OF_DEVICES; devices++)
        {
            if (AMFITRACK.getDeviceActive(devices))
            {
                AMFITRACK.checkDeviceDisconnected(devices);
            }
        }

        AMFITRACK.processConfigurationEnumeration();

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
#endif
}

void AMFITRACK::start_amfitrack_task(void)
{
#if defined(USE_THREAD_BASED)
    stop_running = false;

#ifdef AMFITRACK_DEBUG_INFO
    std::cout << "Starting Background thread!" << std::endl;
#endif // AMFITRACK_DEBUG_INFO

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
    usb_connection &usb = usb_connection::getInstance();
#endif
    AmfiProt_API &amfiprot_api = AmfiProt_API::getInstance();
    AMFITRACK &AMFITRACK = AMFITRACK::getInstance();

#ifdef USE_USB
    usb.usb_run();
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

    AMFITRACK.processConfigurationEnumeration();
}

void AMFITRACK::initialize_amfitrack()
{
#ifdef USE_USB
    /* Initialize USB conenction */
    usb_connection &usb = usb_connection::getInstance();
#endif

#ifdef USE_USB
    usb.usb_init();
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

#ifdef AMFITRACK_DEBUG_INFO
    std::cout << Name[DeviceID] << std::endl;
#endif // AMFITRACK_DEBUG_INFO
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

//-----------------------------------------------------------------------------
// Configuration enumeration (full-config read)
//-----------------------------------------------------------------------------

namespace
{
constexpr int kConfigRequestTimeoutMs = 250;
constexpr uint8_t kConfigMaxRetries = 8;
} // namespace

void AMFITRACK::sendCountRequest(uint8_t DeviceID)
{
    AmfiProt_API &amfiprot_api = AmfiProt_API::getInstance();
    lib_AmfiProt_ConfigValueCountRequest_t req = {};
    req.payloadID = lib_AmfiProt_PayloadID_RequestConfigurationValueCount;
    req.category = 0xFF;
    amfiprot_api.queue_frame(&req, sizeof(req), libAmfiProt_PayloadType_Common, lib_AmfiProt_packetType_NoAck, DeviceID);
}

void AMFITRACK::sendNameAndUidRequest(uint8_t DeviceID, uint16_t index)
{
    AmfiProt_API &amfiprot_api = AmfiProt_API::getInstance();
    lib_AmfiProt_ConfigNameRequestUID_t req = {};
    req.payloadID = lib_AmfiProt_PayloadID_RequestConfigurationNameAndUID;
    req.category = 0xFF;
    req.index = index;
    amfiprot_api.queue_frame(&req, sizeof(req), libAmfiProt_PayloadType_Common, lib_AmfiProt_packetType_NoAck, DeviceID);
}

void AMFITRACK::sendValueRequest(uint8_t DeviceID, uint32_t uid)
{
    AmfiProt_API &amfiprot_api = AmfiProt_API::getInstance();
    lib_AmfiProt_ConfigValueUIDRequest_t req = {};
    req.payloadID = lib_AmfiProt_PayloadID_RequestConfigurationValueUID;
    req.uid = uid;
    amfiprot_api.queue_frame(&req, sizeof(req), libAmfiProt_PayloadType_Common, lib_AmfiProt_packetType_NoAck, DeviceID);
}

void AMFITRACK::failEnumeration(ConfigEnumState &st)
{
    st.status = ConfigEnumStatus::Failed;
    st.waiting_for = ConfigWaitingFor::Nothing;
}

bool AMFITRACK::requestFullConfiguration(uint8_t DeviceID)
{
    if (!getDeviceActive(DeviceID))
        return false;

    {
#ifdef USE_THREAD_BASED
        const std::lock_guard<std::mutex> lock(mutConfigEnum);
#endif
        ConfigEnumState &st = ConfigEnumStates[DeviceID];
        if (st.status == ConfigEnumStatus::InProgress)
            return false;
        st.parameters.clear();
        st.status = ConfigEnumStatus::InProgress;
        st.waiting_for = ConfigWaitingFor::Count;
        st.expected_count = 0;
        st.pending_index = 0;
        st.pending_uid = 0;
        st.retry_count = 0;
        st.last_request_time = std::chrono::steady_clock::now();
    }
    sendCountRequest(DeviceID);
    return true;
}

ConfigEnumStatus AMFITRACK::getConfigurationStatus(uint8_t DeviceID)
{
#ifdef USE_THREAD_BASED
    const std::lock_guard<std::mutex> lock(mutConfigEnum);
#endif
    return ConfigEnumStates[DeviceID].status;
}

bool AMFITRACK::getFullConfiguration(uint8_t DeviceID, std::vector<ConfigParameter> &out)
{
#ifdef USE_THREAD_BASED
    const std::lock_guard<std::mutex> lock(mutConfigEnum);
#endif
    ConfigEnumState &st = ConfigEnumStates[DeviceID];
    if (st.status != ConfigEnumStatus::Complete)
        return false;
    out = st.parameters;
    return true;
}

void AMFITRACK::onConfigCountReply(uint8_t DeviceID, uint16_t count)
{
    bool send_next = false;
    uint16_t next_index = 0;
    {
#ifdef USE_THREAD_BASED
        const std::lock_guard<std::mutex> lock(mutConfigEnum);
#endif
        ConfigEnumState &st = ConfigEnumStates[DeviceID];
        if (st.status != ConfigEnumStatus::InProgress || st.waiting_for != ConfigWaitingFor::Count)
            return;
        st.expected_count = count;
        if (count == 0)
        {
            st.status = ConfigEnumStatus::Complete;
            st.waiting_for = ConfigWaitingFor::Nothing;
            return;
        }
        st.parameters.assign(count, ConfigParameter{});
        st.waiting_for = ConfigWaitingFor::NameAndUid;
        st.pending_index = 0;
        st.retry_count = 0;
        st.last_request_time = std::chrono::steady_clock::now();
        send_next = true;
        next_index = 0;
    }
    if (send_next)
        sendNameAndUidRequest(DeviceID, next_index);
}

void AMFITRACK::onConfigNameAndUidReply(uint8_t DeviceID, uint16_t index, uint8_t category, uint32_t uid, const char *name, size_t name_len)
{
    bool send_name = false;
    bool send_value = false;
    uint16_t next_index = 0;
    uint32_t next_uid = 0;
    {
#ifdef USE_THREAD_BASED
        const std::lock_guard<std::mutex> lock(mutConfigEnum);
#endif
        ConfigEnumState &st = ConfigEnumStates[DeviceID];
        if (st.status != ConfigEnumStatus::InProgress || st.waiting_for != ConfigWaitingFor::NameAndUid)
            return;
        if (index != st.pending_index || index >= st.parameters.size())
            return; // stale or out-of-range; rely on retry

        ConfigParameter &p = st.parameters[index];
        p.uid = uid;
        p.category = category;
        size_t copy_len = name_len < (CONFIG_PARAM_NAME_LENGTH - 1) ? name_len : (CONFIG_PARAM_NAME_LENGTH - 1);
        memcpy(p.name, name, copy_len);
        p.name[copy_len] = '\0';
        p.name_received = true;

        st.retry_count = 0;
        st.last_request_time = std::chrono::steady_clock::now();

        if (index + 1 < st.expected_count)
        {
            st.pending_index = index + 1;
            next_index = st.pending_index;
            send_name = true;
        }
        else
        {
            st.waiting_for = ConfigWaitingFor::Value;
            st.pending_index = 0;
            st.pending_uid = st.parameters[0].uid;
            next_uid = st.pending_uid;
            send_value = true;
        }
    }
    if (send_name)
        sendNameAndUidRequest(DeviceID, next_index);
    if (send_value)
        sendValueRequest(DeviceID, next_uid);
}

void AMFITRACK::onConfigValueUidReply(uint8_t DeviceID, uint32_t uid, const lib_Generic_Parameter_Value_t &value)
{
    bool send_next = false;
    uint32_t next_uid = 0;
    {
#ifdef USE_THREAD_BASED
        const std::lock_guard<std::mutex> lock(mutConfigEnum);
#endif
        ConfigEnumState &st = ConfigEnumStates[DeviceID];
        if (st.status != ConfigEnumStatus::InProgress || st.waiting_for != ConfigWaitingFor::Value)
            return;
        if (st.pending_index >= st.parameters.size() || st.parameters[st.pending_index].uid != uid)
            return; // stale or duplicate

        st.parameters[st.pending_index].value = value;
        st.parameters[st.pending_index].value_received = true;

        st.retry_count = 0;
        st.last_request_time = std::chrono::steady_clock::now();

        if (st.pending_index + 1 < st.expected_count)
        {
            st.pending_index += 1;
            st.pending_uid = st.parameters[st.pending_index].uid;
            next_uid = st.pending_uid;
            send_next = true;
        }
        else
        {
            st.status = ConfigEnumStatus::Complete;
            st.waiting_for = ConfigWaitingFor::Nothing;
        }
    }
    if (send_next)
        sendValueRequest(DeviceID, next_uid);
}

void AMFITRACK::processConfigurationEnumeration()
{
    auto now = std::chrono::steady_clock::now();
    for (uint8_t id = 0; id < MAX_NUMBER_OF_DEVICES; id++)
    {
        ConfigWaitingFor what = ConfigWaitingFor::Nothing;
        uint16_t resend_index = 0;
        uint32_t resend_uid = 0;
        bool resend = false;
        {
#ifdef USE_THREAD_BASED
            const std::lock_guard<std::mutex> lock(mutConfigEnum);
#endif
            ConfigEnumState &st = ConfigEnumStates[id];
            if (st.status != ConfigEnumStatus::InProgress)
                continue;
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - st.last_request_time);
            if (elapsed.count() < kConfigRequestTimeoutMs)
                continue;
            if (st.retry_count >= kConfigMaxRetries)
            {
                failEnumeration(st);
                continue;
            }
            st.retry_count++;
            st.last_request_time = now;
            what = st.waiting_for;
            resend_index = st.pending_index;
            resend_uid = st.pending_uid;
            resend = true;
        }
        if (!resend)
            continue;
        switch (what)
        {
        case ConfigWaitingFor::Count:
            sendCountRequest(id);
            break;
        case ConfigWaitingFor::NameAndUid:
            sendNameAndUidRequest(id, resend_index);
            break;
        case ConfigWaitingFor::Value:
            sendValueRequest(id, resend_uid);
            break;
        case ConfigWaitingFor::Nothing:
            break;
        }
    }
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
        std::cout << "Device " << std::dec << static_cast<unsigned>(DeviceID) << " disconnected" << std::endl;
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
        std::cout << "Device " << std::dec << static_cast<unsigned>(DeviceID) << " connected" << std::endl;
    DeviceActive[DeviceID] = true;
    DeviceLastTimeSeen[DeviceID] = time(0);
#ifdef AMFITRACK_DEBUG_INFO
    std::cout << "Device " << DeviceID << " is active" << std::endl;
#endif // AMFITRACK_DEBUG_INFO
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
#ifdef AMFITRACK_DEBUG_INFO
    std::cout << "Pose set!" << std::endl;
    // printf("Pose X %.3f | Y %.3f | Z %.3f \n", this->Pose[DeviceID].position_x_in_m, this->Pose[DeviceID].position_y_in_m, this->Pose[DeviceID].position_z_in_m);
#endif // AMFITRACK_DEBUG_INFO
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

void AMFITRACK::setDeviceUUID(uint8_t DeviceID, uint32_t UUID[3])
{
#ifdef USE_THREAD_BASED
    const std::lock_guard<std::mutex> lock(mutUUID);
#endif // USE_THREAD_BASED
    std::ostringstream uuid_stream;
    uuid_stream << std::hex << std::uppercase << std::setfill('0');
    uuid_stream << std::setw(8) << UUID[0];
    uuid_stream << std::setw(8) << UUID[1];
    uuid_stream << std::setw(8) << UUID[2];
    std::string uuid_string = uuid_stream.str();
    for (uint8_t i = 0; i < 24; i++)
    {
        DeviceUUID[DeviceID][i] = uuid_string[i];
    }
    DeviceUUID[DeviceID][24] = '\0';
}

void AMFITRACK::getDeviceUUID(uint8_t DeviceID, char *UUID)
{
    if (!getDeviceActive(DeviceID))
        return;
#ifdef USE_THREAD_BASED
    const std::lock_guard<std::mutex> lock(mutUUID);
#endif // USE_THREAD_BASED
    memcpy(UUID, DeviceUUID[DeviceID], UUID_LENGTH + 1);
}

void AMFITRACK::setDeviceType(uint8_t DeviceID, DeviceType type)
{
#ifdef USE_THREAD_BASED
    const std::lock_guard<std::mutex> lock(mutDeviceActive);
#endif // USE_THREAD_BASED
    DeviceTypes[DeviceID] = type;
}

AMFITRACK::DeviceType AMFITRACK::getDeviceType(uint8_t DeviceID)
{
#ifdef USE_THREAD_BASED
    const std::lock_guard<std::mutex> lock(mutDeviceActive);
#endif // USE_THREAD_BASED
    return DeviceTypes[DeviceID];
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_SourceCalibration(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)routing_handle;
    AMFITRACK &AMFITRACK = AMFITRACK::getInstance();
    AMFITRACK.setDeviceActive(frame->header.source);
    AMFITRACK.setDeviceType(frame->header.source, AMFITRACK::DeviceType::Source);
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_SourceMeasurement(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)routing_handle;
    AMFITRACK &AMFITRACK = AMFITRACK::getInstance();
    AMFITRACK.setDeviceActive(frame->header.source);
    AMFITRACK.setDeviceType(frame->header.source, AMFITRACK::DeviceType::Source);
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_SensorMeasurement(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)routing_handle;
    lib_AmfiProt_Amfitrack_Sensor_Measurement_t SensorMeasurement;
    AMFITRACK &AMFITRACK = AMFITRACK::getInstance();
    memcpy(&SensorMeasurement, &frame->payload[0], sizeof(lib_AmfiProt_Amfitrack_Sensor_Measurement_t));
    lib_AmfiProt_Amfitrack_Pose_t tempPose;
    lib_AmfiProt_Amfitrack_decode_pose_i24(&SensorMeasurement.pose, &tempPose);
    AMFITRACK.setDevicePose(frame->header.source, tempPose);
    lib_AmfiProt_Amfitrack_IMU_t tempIMU;
    lib_AmfiProt_Amfitrack_decodeIMU_i16(&SensorMeasurement.imu_data, &tempIMU);
    AMFITRACK.setDeviceIMU(frame->header.source, tempIMU);
    AMFITRACK.setSensorMeasurements(frame->header.source, SensorMeasurement);
    AMFITRACK.setDeviceActive(frame->header.source);
    AMFITRACK.setDeviceType(frame->header.source, AMFITRACK::DeviceType::Sensor);
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_RawBfield(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_NormalizedBfield(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_BfieldPhase(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_NormalizedBfieldImu(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_SignData(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_PllData(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_RawFloats(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_SetPhaseModulation(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_SourceCoilCalData(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_AlternativeProcessing(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RequestProcedureSpec(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_ReplyProcedureSpec(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RequestProcedureCall(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_ReplyProcedureCall(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RequestDeviceID(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RespondDeviceID(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)routing_handle;
    AMFITRACK &AMFITRACK = AMFITRACK::getInstance();
    AMFITRACK.setDeviceActive(frame->header.source);
    lib_AmfiProt_DeviceID payload;
    memcpy(&payload, &frame->payload[0], sizeof(lib_AmfiProt_DeviceID));
    uint32_t UUID[3]{ 0,0,0 };
    memcpy(&UUID, &payload.UUID, sizeof(uint32_t) * 3);
    AMFITRACK.setDeviceUUID(frame->header.source, UUID);
}

void AmfiProt_API::libAmfiProt_handle_SetTxID(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RequestFirmwareVersion(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_FirmwareVersion(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_FirmwareStart(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_FirmwareData(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_FirmwareEnd(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RequestDeviceName(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_ReplyDeviceName(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)routing_handle;
    AMFITRACK &AMFITRACK = AMFITRACK::getInstance();
    size_t str_length = strnlen((char *)(&(frame->payload[1])), MAX_PAYLOAD_SIZE - 1);
    AMFITRACK.setDeviceName(frame->header.source, (char *)(&(frame->payload[1])), (uint8_t)str_length);
}

void AmfiProt_API::libAmfiProt_handle_RequestConfigurationValue(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_ReplyConfigurationValue(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_SetConfigurationValue(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RequestConfigurationName(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_ReplyConfigurationName(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_LoadDefault(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_SaveAsDefault(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RequestConfigurationNameAndUID(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_ConfigurationNameAndUID(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)routing_handle;
    AMFITRACK &AMFITRACK = AMFITRACK::getInstance();
    // Wire layout: [payloadID(1)] [index(u16 LE)] [category(u8)] [uid(u32 LE)] [name...]
    if (frame->header.length < 1 + 2 + 1 + 4 + 1)
        return;
    uint16_t index;
    uint8_t category;
    uint32_t uid;
    memcpy(&index, &frame->payload[1], sizeof(index));
    memcpy(&category, &frame->payload[3], sizeof(category));
    memcpy(&uid, &frame->payload[4], sizeof(uid));
    const char *name = reinterpret_cast<const char *>(&frame->payload[8]);
    size_t name_max = static_cast<size_t>(frame->header.length) - 8;
    size_t name_len = strnlen(name, name_max);
    AMFITRACK.onConfigNameAndUidReply(frame->header.source, index, category, uid, name, name_len);
}

void AmfiProt_API::libAmfiProt_handle_RequestConfigurationValueUID(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_ConfigurationValueUID(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)routing_handle;
    AMFITRACK &AMFITRACK = AMFITRACK::getInstance();
    // Wire layout: [payloadID(1)] [uid(u32 LE)] [value (type byte + variable data)]
    if (frame->header.length < 1 + 4 + 1)
        return;
    uint32_t uid;
    memcpy(&uid, &frame->payload[1], sizeof(uid));
    lib_Generic_Parameter_Value_t value = {};
    size_t value_bytes = static_cast<size_t>(frame->header.length) - 5;
    if (value_bytes > sizeof(value))
        value_bytes = sizeof(value);
    memcpy(&value, &frame->payload[5], value_bytes);
    AMFITRACK.onConfigValueUidReply(frame->header.source, uid, value);
}

void AmfiProt_API::libAmfiProt_handle_SetConfigurationValueUID(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RequestConfigurationCategory(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_ConfigurationCategory(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RequestConfigurationValueCount(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_ConfigurationValueCount(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)routing_handle;
    AMFITRACK &AMFITRACK = AMFITRACK::getInstance();
    lib_AmfiProt_ConfigValueCount_t payload = {};
    memcpy(&payload, &frame->payload[0], sizeof(payload));
    AMFITRACK.onConfigCountReply(frame->header.source, payload.count);
}

void AmfiProt_API::libAmfiProt_handle_RequestCategoryCount(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_CategoryCount(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_Reboot(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_DebugOutput(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_ResetParameter(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RequestFirmwareVersionPerID(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
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

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_SensorMeasurement(void *handle, lib_AmfiProt_Frame_t *frame, std::chrono::steady_clock::time_point time_stamp, void *routing_handle)
{
    AMFITRACK &AMFITRACK = AMFITRACK::getInstance();
    AMFITRACK.setSensorTimestamp(frame->header.source, time_stamp);
    AmfiProt_API::lib_AmfiProt_Amfitrack_handle_SensorMeasurement(handle, frame, routing_handle);
}

#endif