#include <iostream>
#include <cstdint>
#include <fstream>
#include <string>
#include <ctime>
#include "lib_AmfiProt_API.hpp"
#include "Amfitrack.hpp"
#include "src/usb_connection.h"
#include <process.h>
#include <sstream>
#include <iomanip>

using namespace AMFITRACK_API_LIB;

//#define AMFITRACK_DEBUG_INFO

static bool stop_running = false;

AMFITRACK::AMFITRACK()
{
    // Initialize Name with null characters
    for (int i = 0; i < AMFITRACK_MAX_NUMBER_OF_DEVICES; i++)
    {
        memset(Name[i], 0, AMFITRACK_MAX_NAME_LENGTH);
        memset(DeviceUUID[i], 0, AMFITRACK_UUID_LENGTH);
        DeviceUUID_Number[i] = 0;
        DeviceActive[i] = false;
        Position[i] = { .position_x_in_m = 0, .position_y_in_m = 0, .position_z_in_m = 0, .orientation_x = 0, .orientation_y = 0, .orientation_z = 0, .orientation_w = 0 };
        
    }
    for (int i = 0; i < MEDABILITY_SENSOR_SLOT_COUNT; i++)
    {
        SlotDeviceChanged[i] = false;
        DeviceSlot[i] = 0;
    }
}


AMFITRACK::~AMFITRACK()
{

}

void AMFITRACK::background_amfitrack_task(AMFITRACK* inst)
{
    /* Creates instance of USB */
    usb_connection& usb = usb_connection::getInstance();
    AmfiProt_API& amfiprot_api = AmfiProt_API::getInstance();
    AMFITRACK& AMFITRACK = AMFITRACK::getInstance();

#ifdef AMFITRACK_DEBUG_INFO
    std::cout << "Background thread started!" << std::endl;
#endif // AMFITRACK_DEBUG_INFO

    while (!stop_running)
    {
        usb.usb_run();
        amfiprot_api.amfiprot_run();

        for (uint8_t devices = 0; devices < AMFITRACK_MAX_NUMBER_OF_DEVICES; devices++)
        {
            if (AMFITRACK.getDeviceActive(devices))
            {
                AMFITRACK.checkDeviceDisconnected(devices);
            }
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}


void AMFITRACK::start_amfitrack_task(void)
{
    stop_running = false;

#ifdef AMFITRACK_DEBUG_INFO
    std::cout << "Starting Background thread!" << std::endl;
#endif // AMFITRACK_DEBUG_INFO

    // Create a thread object
    std::thread background_thread(background_amfitrack_task, this);

    background_thread.detach();
}

void AMFITRACK::stop_amfitrack_task(void)
{
    stop_running = true;
}

void AMFITRACK::initialize_amfitrack(void)
{
    usb_connection& usb = usb_connection::getInstance();
    AmfiProt_API& amfiprot_api = AmfiProt_API::getInstance();
    /* Initialize USB conenction */
    usb.usb_init();
}

void AMFITRACK::setDeviceName(uint8_t DeviceID, char* name, uint8_t length)
{
    
    // Check for valid device ID and name length
    if (length >= AMFITRACK_MAX_NAME_LENGTH) return;
    for (uint8_t i = 0; i < length; i++)
    {
        Name[DeviceID][i] = name[i];
    }
    if (name == std::string("Amfitrack Sensor"))
    {
        if (std::find(ids_sensor.begin(), ids_sensor.end(), DeviceID) == ids_sensor.end())
        {
            ids_sensor.push_back(DeviceID);
        }
    }
    else if (name == std::string("Amfitrack RF Hub"))
        id_rfhub = DeviceID;
    else if (std::string(name).find("Amfitrack Source") != std::string::npos)
        id_emfsource = DeviceID;
        
    Name[DeviceID][length] = '\0'; // Ensure null termination

#ifdef AMFITRACK_DEBUG_INFO
    std::cout << Name[DeviceID] << std::endl;
#endif // AMFITRACK_DEBUG_INFO
}

std::string AMFITRACK_API_LIB::AMFITRACK::getDeviceName(uint8_t DeviceID)
{
    return (Name[DeviceID]);
}

void AMFITRACK::checkDeviceDisconnected(uint8_t DeviceID)
{
    time_t CurrentTime = time(0);

    if (difftime(CurrentTime, DeviceLastTimeSeen[DeviceID]) > 5.0)
    {
        DeviceActive[DeviceID] = false;
        std::cout << "Device " << std::dec << static_cast<unsigned>(DeviceID) << " disconnected" << std::endl;
    }

}

void AMFITRACK::setDeviceActive(uint8_t DeviceID)
{
    if (!DeviceActive[DeviceID])
    {
        std::cout << "Device " << std::dec << static_cast<unsigned>(DeviceID) << " connected" << std::endl;
        if (DeviceID != id_emfsource && DeviceID != id_rfhub)
        {
            for (int i = 0; i < MEDABILITY_SENSOR_SLOT_COUNT; i++)
            {
                if (DeviceSlot[i] == DeviceID)
                {
                    SlotDeviceChanged[i];
                    break;
                }
            }
        }
    }
    DeviceActive[DeviceID] = true;
    DeviceLastTimeSeen[DeviceID] = time(0);
#ifdef AMFITRACK_DEBUG_INFO
    std::cout << "Device " << DeviceID << " is active" << std::endl;
#endif // AMFITRACK_DEBUG_INFO

}

bool AMFITRACK::getDeviceActive(uint8_t DeviceID)
{
    return DeviceActive[DeviceID];
}

void AMFITRACK::setDevicePose(uint8_t DeviceID, lib_AmfiProt_Amfitrack_Pose_t Pose)
{
    memcpy(&Position[DeviceID], &Pose, sizeof(lib_AmfiProt_Amfitrack_Pose_t));
#ifdef AMFITRACK_DEBUG_INFO
    std::cout << "Pose set!" << std::endl;
    //printf("Pose X %.3f | Y %.3f | Z %.3f \n", this->Pose[DeviceID].position_x_in_m, this->Pose[DeviceID].position_y_in_m, this->Pose[DeviceID].position_z_in_m);
#endif // AMFITRACK_DEBUG_INFO

}

void AMFITRACK::getDevicePose(uint8_t DeviceID, lib_AmfiProt_Amfitrack_Pose_t* Pose)
{
    if (!getDeviceActive(DeviceID)) return;
    memcpy(Pose, &Position[DeviceID], sizeof(lib_AmfiProt_Amfitrack_Pose_t));
}

void AMFITRACK_API_LIB::AMFITRACK::setSensorSlot(uint8_t slot, uint8_t DeviceID)
{
    DeviceSlot[slot] = DeviceID;
    SlotDeviceChanged[slot] = true;
}

uint8_t AMFITRACK_API_LIB::AMFITRACK::getSensorInSlot(uint8_t slot)
{
    if (slot >= MEDABILITY_SENSOR_SLOT_COUNT)
        return 0;
    return DeviceSlot[slot];
}




void AMFITRACK::getDeviceTemperature(uint8_t DeviceID, float* temp)
{
    if (!getDeviceActive(DeviceID)) return;
    memcpy(temp, &Temperature[DeviceID], sizeof(float));
}
void AMFITRACK::setDeviceTemperature(uint8_t DeviceID, float temp)
{
    memcpy(&Temperature[DeviceID], &temp, sizeof(float));
}

void AMFITRACK_API_LIB::AMFITRACK::setDeviceUUID(uint8_t DeviceID, uint32_t UUID[3])
{
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
    DeviceUUID_Number[DeviceID] = UUID[0];

}

std::string AMFITRACK_API_LIB::AMFITRACK::getDeviceUUID(uint8_t DeviceID)
{
    return DeviceUUID[DeviceID];
}

uint32_t AMFITRACK_API_LIB::AMFITRACK::getDeviceUUIDShort(uint8_t DeviceID)
{
    if (DeviceUUID_Number[DeviceID] == 0 && getDeviceActive(DeviceID))
    {
        auto& usb_conn = usb_connection::getInstance();
        usb_conn.find_nodes();
    }
    return DeviceUUID_Number[DeviceID];
}

bool AMFITRACK_API_LIB::AMFITRACK::getSlotDeviceChanged(uint8_t DeviceID)
{
    return SlotDeviceChanged[DeviceID];
}

void AMFITRACK_API_LIB::AMFITRACK::resetSlotDeviceChanged(uint8_t DeviceID)
{
    SlotDeviceChanged[DeviceID] = 0;
}


std::vector<uint8_t> AMFITRACK_API_LIB::AMFITRACK::getSensorIDs()
{
    return ids_sensor;
}

uint32_t AMFITRACK_API_LIB::AMFITRACK::getRFHubSerial()
{
    return DeviceUUID_Number[id_rfhub];
}
uint32_t AMFITRACK_API_LIB::AMFITRACK::getEMFSourceSerial()
{
    return DeviceUUID_Number[id_emfsource];
}

uint8_t AMFITRACK_API_LIB::AMFITRACK::getEMFSourceID()
{
    return id_emfsource;
}

std::vector<uint8_t> AMFITRACK_API_LIB::AMFITRACK::getActiveDevices()
{
    
    std::vector<uint8_t> active_ids;
    for (uint8_t i = 0; i < AMFITRACK_MAX_NUMBER_OF_DEVICES; i++)
    {
        if (DeviceActive[i])
            active_ids.push_back(i);
    }
    return active_ids;
}


void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_SourceCalibration(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    AMFITRACK& AMFITRACK = AMFITRACK::getInstance();
    AMFITRACK.setDeviceActive(frame->header.source);
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_SourceMeasurement(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    AMFITRACK& AMFITRACK = AMFITRACK::getInstance();
    AMFITRACK.setDeviceActive(frame->header.source);
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_SensorMeasurement(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    AMFITRACK& AMFITRACK = AMFITRACK::getInstance();
    lib_AmfiProt_Amfitrack_Sensor_Measurement_t SensorMeasurement;
    memcpy(&SensorMeasurement, &frame->payload[0], sizeof(lib_AmfiProt_Amfitrack_Sensor_Measurement_t));
    lib_AmfiProt_Amfitrack_Pose_t tempPose;
    lib_AmfiProt_Amfitrack_decode_pose_i24(&SensorMeasurement.pose, &tempPose);
    float temp_in_c = static_cast<float>(SensorMeasurement.temperature) / 2.f - 30.f;
    AMFITRACK.setDeviceTemperature(frame->header.source, temp_in_c);
    AMFITRACK.setDevicePose(frame->header.source, tempPose);
    AMFITRACK.setDeviceActive(frame->header.source);
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_RawBfield(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_NormalizedBfield(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_BfieldPhase(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_NormalizedBfieldImu(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_SignData(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_PllData(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_RawFloats(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_SetPhaseModulation(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_SourceCoilCalData(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_AlternativeProcessing(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}


void AmfiProt_API::libAmfiProt_handle_RequestProcedureSpec(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_ReplyProcedureSpec(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RequestProcedureCall(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_ReplyProcedureCall(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RequestDeviceID(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RespondDeviceID(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    AMFITRACK& AMFITRACK = AMFITRACK::getInstance();
    lib_AmfiProt_DeviceID payload;
    memcpy(&payload, &frame->payload[0], sizeof(lib_AmfiProt_DeviceID));
    uint32_t UUID[3]{ 0,0,0 };
    memcpy(&UUID, &payload.UUID, sizeof(uint32_t) * 3);
    AMFITRACK.setDeviceActive(frame->header.source);
    AMFITRACK.setDeviceUUID(frame->header.source, UUID);
}

void AmfiProt_API::libAmfiProt_handle_SetTxID(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RequestFirmwareVersion(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_FirmwareVersion(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_FirmwareStart(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_FirmwareData(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_FirmwareEnd(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RequestDeviceName(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_ReplyDeviceName(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    AMFITRACK& AMFITRACK = AMFITRACK::getInstance();
    size_t str_length = strnlen((char*)(&(frame->payload[1])), MAX_PAYLOAD_SIZE - 1);
    AMFITRACK.setDeviceName(frame->header.source, (char*)(&(frame->payload[1])), str_length);
}

void AmfiProt_API::libAmfiProt_handle_RequestConfigurationValue(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_ReplyConfigurationValue(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_SetConfigurationValue(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RequestConfigurationName(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_ReplyConfigurationName(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_LoadDefault(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_SaveAsDefault(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RequestConfigurationNameAndUID(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_ConfigurationNameAndUID(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RequestConfigurationValueUID(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_ConfigurationValueUID(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_SetConfigurationValueUID(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RequestConfigurationCategory(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_ConfigurationCategory(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RequestConfigurationValueCount(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_ConfigurationValueCount(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RequestCategoryCount(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_CategoryCount(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_Reboot(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_DebugOutput(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_ResetParameter(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}

void AmfiProt_API::libAmfiProt_handle_RequestFirmwareVersionPerID(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    /* NOTE: Overwrite in application-specific library */
}



extern "C"
{
    __declspec(dllexport) void InitializeAmfitrack()
    {
        AMFITRACK_API_LIB::AMFITRACK::getInstance().initialize_amfitrack();
    }

    __declspec(dllexport) void StartAmfitrackTask()
    {
        AMFITRACK_API_LIB::AMFITRACK::getInstance().start_amfitrack_task();
    }

    __declspec(dllexport) void StopAmfitrackTask()
    {
        AMFITRACK_API_LIB::AMFITRACK::getInstance().stop_amfitrack_task();
    }

    __declspec(dllexport) bool GetDevicePose(uint8_t id, lib_AmfiProt_Amfitrack_Pose_t &buffer)
    {
        AMFITRACK_API_LIB::AMFITRACK::getInstance().getDevicePose(id, &buffer);
        return true;
    }

    __declspec(dllexport) uint8_t GetActiveDevice()
    {
        auto devices = AMFITRACK_API_LIB::AMFITRACK::getInstance().getActiveDevices();
        return devices[0];
    }
}
