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
#include "lib_AmfiProt_API.hpp"
#include "Amfitrack.h"
#include "AmfitrackDeviceTypes.h"
#include "Amfitrack_config.h"
#include "Amfitrack_Devices.h"
#include "lib_log.h"
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
	lib_AmfiProt_Amfitrack_Source_Calibration_t sourceCalibration;
	memcpy(&sourceCalibration, &frame->payload[0], sizeof(lib_AmfiProt_Amfitrack_Source_Calibration_t));

	uint8_t _deviceID = frame->header.source;
	Frequency_t freq;
	Calibration_t cal;
	freq.Frequency_X = sourceCalibration.frequency_x_in_Hz;
	freq.Frequency_Y = sourceCalibration.frequency_y_in_Hz;
	freq.Frequency_Z = sourceCalibration.frequency_z_in_Hz;

	cal.Calibration_X = sourceCalibration.calibration_source_coil_x;
	cal.Calibration_Y = sourceCalibration.calibration_source_coil_y;
	cal.Calibration_Z = sourceCalibration.calibration_source_coil_z;

	AMFITRACK_Devices::getInstance().set(_deviceID, freq);
	AMFITRACK_Devices::getInstance().set(_deviceID, cal);
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_SourceMeasurement(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
	(void)handle;
	(void)routing_handle;
	lib_AmfiProt_Amfitrack_Source_Measurement_t sourceMeasurement;
	memcpy(&sourceMeasurement, &frame->payload[0], sizeof(lib_AmfiProt_Amfitrack_Source_Measurement_t));

	uint8_t _deviceID = frame->header.source;
	Current_t cur;
	Voltage_t vol;
	cur.Current_X = sourceMeasurement.current_coil_x_in_mA / 1000;
	cur.Current_Y = sourceMeasurement.current_coil_y_in_mA / 1000;
	cur.Current_Z = sourceMeasurement.current_coil_z_in_mA / 1000;

	vol.Voltage_X = sourceMeasurement.voltage_coil_x_in_V;
	vol.Voltage_Y = sourceMeasurement.voltage_coil_y_in_V;
	vol.Voltage_Z = sourceMeasurement.voltage_coil_z_in_V;
	vol.Voltage_Boost = sourceMeasurement.voltage_boost_in_V;

	AMFITRACK_Devices::getInstance().set(_deviceID, cur);
	AMFITRACK_Devices::getInstance().set(_deviceID, vol);
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_SensorMeasurement(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
	(void)handle;
	(void)routing_handle;
	uint8_t _deviceID = frame->header.source;

	lib_AmfiProt_Amfitrack_Sensor_Measurement_t SensorMeasurement;
	memcpy(&SensorMeasurement, &frame->payload[0], sizeof(lib_AmfiProt_Amfitrack_Sensor_Measurement_t));
	lib_AmfiProt_Amfitrack_Pose_t tempPose;
	lib_AmfiProt_Amfitrack_decode_pose_i24(&SensorMeasurement.pose, &tempPose);
	lib_AmfiProt_Amfitrack_IMU_t tempIMU;
	lib_AmfiProt_Amfitrack_decodeIMU_i16(&SensorMeasurement.imu_data, &tempIMU);

	Pose_t pose = { tempPose.position_x_in_m, tempPose.position_y_in_m, tempPose.position_z_in_m, tempPose.orientation_x, tempPose.orientation_y, tempPose.orientation_z, tempPose.orientation_w };
	AMFITRACK_Devices::getInstance().set(_deviceID, pose);
	IMU_t imu;
	imu.Acceleration_X = tempIMU.acceleration_x_in_mg / 1000;
	imu.Acceleration_Y = tempIMU.acceleration_y_in_mg / 1000;
	imu.Acceleration_Z = tempIMU.acceleration_z_in_mg / 1000;
	imu.Rotation_X = tempIMU.rotation_x_in_rad_per_sec;
	imu.Rotation_Y = tempIMU.rotation_y_in_rad_per_sec;
	imu.Rotation_Z = tempIMU.rotation_z_in_rad_per_sec;
	AMFITRACK_Devices::getInstance().set(_deviceID, imu);
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_SensorStatus(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
	(void)handle;
	(void)routing_handle;
	uint8_t _deviceID = frame->header.source;
	lib_AmfiProt_Amfitrack_Sensor_Status_t SensorStatus;
	memcpy(&SensorStatus, &frame->payload[0], sizeof(lib_AmfiProt_Amfitrack_Sensor_Status_t));
	Status_t status;
	status.Battery_SOC = SensorStatus.bat_SOC;
	AMFITRACK_Devices::getInstance().set(_deviceID, status);
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_RawBfield(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
	(void)handle;
	(void)routing_handle;
	uint8_t _deviceID = frame->header.source;
	lib_AmfiProt_Amfitrack_Sensor_BField_t bfield;
	memcpy(&bfield, &frame->payload[0], sizeof(lib_AmfiProt_Amfitrack_Sensor_BField_t));
	Raw_B_Field_t rawBfield;
	memcpy(&rawBfield.bfield, bfield.bfield, sizeof(bfield.bfield));
	AMFITRACK_Devices::getInstance().set(_deviceID, rawBfield);
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_NormalizedBfield(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
	(void)handle;
	(void)routing_handle;
	uint8_t _deviceID = frame->header.source;
	lib_AmfiProt_Amfitrack_Sensor_BField_t bfield;
	memcpy(&bfield, &frame->payload[0], sizeof(lib_AmfiProt_Amfitrack_Sensor_BField_t));
	Normalized_B_Field_t normBfield;
	memcpy(&normBfield.bfield, bfield.bfield, sizeof(bfield.bfield));
	AMFITRACK_Devices::getInstance().set(_deviceID, normBfield);
}

void AmfiProt_API::lib_AmfiProt_Amfitrack_handle_BfieldPhase(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle)
{
	(void)handle;
	(void)routing_handle;
	uint8_t _deviceID = frame->header.source;
	lib_AmfiProt_Amfitrack_Sensor_BField_With_Phase_t bfield;
	memcpy(&bfield, &frame->payload[0], sizeof(lib_AmfiProt_Amfitrack_Sensor_BField_With_Phase_t));
	Raw_with_Phase_B_Field_t rawWithPhaseBfield;
	memcpy(&rawWithPhaseBfield.bfield, bfield.bfield, sizeof(bfield.bfield));
	AMFITRACK_Devices::getInstance().set(_deviceID, rawWithPhaseBfield);
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
	uint8_t _deviceID = frame->header.source;

	AMFITRACK_Devices::getInstance().set(_deviceID, true);
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
	uint8_t _deviceID = frame->header.source;
	size_t str_length = strnlen((char *)(&(frame->payload[1])), MAX_PAYLOAD_SIZE - 1);

	AMFITRACK_Devices::getInstance().set(_deviceID, (char *)(&(frame->payload[1])), (uint8_t)str_length);
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
	uint8_t _deviceID = frame->header.source;
	lib_AmfiProt_ConfigNameUID_protocol_t amfiConfigNameUID;
	memcpy(&amfiConfigNameUID, frame->payload, frame->header.length);
	AMFITRACK_Config::getInstance().set(_deviceID, amfiConfigNameUID);
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
	uint8_t _deviceID = frame->header.source;
	lib_AmfiProt_ConfigValueUID_t amfiConfigValueUID;
	memcpy(&amfiConfigValueUID, frame->payload, sizeof(amfiConfigValueUID));
	AMFITRACK_Config::getInstance().set(_deviceID, amfiConfigValueUID);
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
	uint8_t _deviceID = frame->header.source;
	lib_AmfiProt_ConfigCategory_t amfiConfigCategory;
	memcpy(&amfiConfigCategory, frame->payload, frame->header.length);
	AMFITRACK_Config::getInstance().set(_deviceID, amfiConfigCategory);
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
	uint8_t _deviceID = frame->header.source;
	lib_AmfiProt_ConfigValueCount_t amfiConfigValueCount;
	memcpy(&amfiConfigValueCount, frame->payload, sizeof(amfiConfigValueCount));
	AMFITRACK_Config::getInstance().set(_deviceID, amfiConfigValueCount);
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
	uint8_t _deviceID = frame->header.source;
	lib_AmfiProt_ConfigCategoryCount_t amfiConfigCategoryCount;
	memcpy(&amfiConfigCategoryCount, frame->payload, sizeof(amfiConfigCategoryCount));
	AMFITRACK_Config::getInstance().set(_deviceID, amfiConfigCategoryCount);
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
