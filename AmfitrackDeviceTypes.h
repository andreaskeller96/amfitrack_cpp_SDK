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
	float Position_X; // in meter
	float Position_Y;
	float Position_Z;
	float Orientation_X;
	float Orientation_Y;
	float Orientation_Z;
	float Orientation_W;
} Pose_t;

typedef struct
{
	float Acceleration_X;
	float Acceleration_Y;
	float Acceleration_Z;
	float Rotation_X;
	float Rotation_Y;
	float Rotation_Z;
} IMU_t;

//-----------------------------------------------------------------------------
// Source struct types
//-----------------------------------------------------------------------------
typedef struct
{
	float Current_X;
	float Current_Y;
	float Current_Z;
} Current_t;

typedef struct
{
	float Voltage_X;
	float Voltage_Y;
	float Voltage_Z;
	float Voltage_Boost;
} Voltage_t;

typedef struct
{
	float Frequency_X;
	float Frequency_Y;
	float Frequency_Z;
} Frequency_t;

typedef struct
{
	float Calibration_X;
	float Calibration_Y;
	float Calibration_Z;
} Calibration_t;

#endif
/** @} */ // end of module
