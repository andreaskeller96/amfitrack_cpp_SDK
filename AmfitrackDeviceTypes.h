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
// Sensor struct types
//-----------------------------------------------------------------------------
typedef struct
{
	float Position_X;
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
