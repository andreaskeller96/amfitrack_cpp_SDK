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
#include "../lib/amfiprotapi/lib_AmfiProt_API.hpp"
#include "../Amfitrack.hpp"
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
void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_SourceCalibration(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)routing_handle;
    AMFITRACK &AMFITRACK = AMFITRACK::getInstance();
    AMFITRACK.setDeviceActive(frame->header.source);
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_SourceMeasurement(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
    (void)handle;
    (void)routing_handle;
    AMFITRACK &AMFITRACK = AMFITRACK::getInstance();
    AMFITRACK.setDeviceActive(frame->header.source);
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
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
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
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
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
    (void)frame;
    (void)routing_handle;
    /* NOTE: Overwrite in application-specific library */
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

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_SensorMeasurement(void *handle, lib_AmfiProt_Frame_t *frame, std::chrono::steady_clock::time_point time_stamp, void *routing_handle)
{
    AMFITRACK &AMFITRACK = AMFITRACK::getInstance();
    AMFITRACK.setSensorTimestamp(frame->header.source, time_stamp);
    AmfiProt_API::lib_AmfiProt_Amfitrack_handle_SensorMeasurement(handle, frame, routing_handle);
}