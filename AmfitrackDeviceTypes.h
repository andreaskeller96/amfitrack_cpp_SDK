//-----------------------------------------------------------------------------
//
//                              AMFITECH APS
//
//                          ALL RIGHTS RESERVED
//
//-----------------------------------------------------------------------------
#pragma once
#ifdef __cplusplus
#include <cstddef>
#include <cstdint>
#include <vector>

#include "lib_Generic_Parameter.h"

//-----------------------------------------------------------------------------
// General struct types
//-----------------------------------------------------------------------------
typedef struct
{
	uint8_t Major;
	uint8_t Minor;
	uint8_t Patch;
	uint8_t Build;
} FW_t;

typedef struct
{
	uint8_t Major;
	uint8_t Minor;
	uint8_t Patch;
	uint8_t Build;
} RF_t;

typedef struct
{
	uint8_t Generation;
	uint8_t Version;
	uint8_t SubVersion;
	uint8_t Frequency;
} HW_t;

//-----------------------------------------------------------------------------
// Configuration
//-----------------------------------------------------------------------------
typedef struct
{
	uint32_t uid;
	char name[45];
	uint8_t categoryIndex;
	lib_Generic_Parameter_Value config;
} ConfigEntry_t;

typedef struct
{
	uint8_t index;
	char name[52];
	uint16_t configCount;
	std::vector<ConfigEntry_t> configs;
} CategoryEntry_t;

typedef struct
{
	uint8_t categoryCount;
	std::vector<CategoryEntry_t> categories;
} DeviceConfig_t;

//-----------------------------------------------------------------------------
// Sensor struct types
//-----------------------------------------------------------------------------
typedef struct
{
	float Position_X;	 // In meter
	float Position_Y;	 // In meter
	float Position_Z;	 // In meter
	float Orientation_X; // In quaternion
	float Orientation_Y; // In quaternion
	float Orientation_Z; // In quaternion
	float Orientation_W; // In quaternion
} Pose_t;

typedef struct
{
	float Acceleration_X; // In g
	float Acceleration_Y; // In g
	float Acceleration_Z; // In g
	float Rotation_X;	  // In radian per second
	float Rotation_Y;	  // In radian per second
	float Rotation_Z;	  // In radian per second
} IMU_t;

typedef struct
{
	float bfield[9]; // In raw ADC count
} Raw_B_Field_t;

typedef struct
{
	float bfield[9]; // In calibrated value
} Normalized_B_Field_t;

typedef struct
{
	float bfield[9]; // In raw ADC count
	float phase[9];
} Raw_with_Phase_B_Field_t;

//-----------------------------------------------------------------------------
// Source struct types
//-----------------------------------------------------------------------------
typedef struct
{
	float Current_X; // In Amp
	float Current_Y; // In Amp
	float Current_Z; // In Amp
} Current_t;

typedef struct
{
	float Voltage_X;	 // In Voltage
	float Voltage_Y;	 // In Voltage
	float Voltage_Z;	 // In Voltage
	float Voltage_Boost; // In Voltage
} Voltage_t;

typedef struct
{
	float Frequency_X; // In Hz
	float Frequency_Y; // In Hz
	float Frequency_Z; // In Hz
} Frequency_t;

typedef struct
{
	float Calibration_X;
	float Calibration_Y;
	float Calibration_Z;
} Calibration_t;

#endif
/** @} */ // end of module
