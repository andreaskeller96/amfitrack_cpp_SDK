//-----------------------------------------------------------------------------
//                              AMFITECH APS
//                          ALL RIGHTS RESERVED
//
// D// $URL: $
// $Rev: $
// $Date: $
// $Author: $
//
// Description
// TODO Write a description here
//
//-----------------------------------------------------------------------------
#pragma once


//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <string.h>
#include <chrono>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "project_conf.h"
#include "lib_AmfiProt_API.hpp"
#ifdef USE_USB
#include "hidapi.h"
#endif
#ifdef USE_THREAD_BASED
#include <mutex>
#endif // USE_THREAD_BASED

#define MAX_NAME_LENGTH 64
#define MAX_NUMBER_OF_DEVICES 254

class AMFITRACK_Sensor
{
public:
    AMFITRACK_Sensor();
    explicit AMFITRACK_Sensor(uint8_t id);

    void reset();

public:
    uint8_t deviceId;
    char name[MAX_NAME_LENGTH];
    uint32_t uuid[3];
    uint8_t FW_Version[4];
    uint8_t RF_Version[4];
    uint8_t HW_Version[4];

    bool active;
    
    uint16_t calcId;

    hid_device *_dev_handle;

    lib_AmfiProt_Amfitrack_Pose_t pose;
    lib_AmfiProt_Amfitrack_IMU_t imu;

#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
    std::chrono::steady_clock::time_point sensorTimestamp;
#endif

    uint32_t lastTimeSeenMs;
};

class AMFITRACK_Source
{
public:
    AMFITRACK_Source();
    explicit AMFITRACK_Source(uint8_t id);

    void reset();

public:
    uint8_t deviceId;
    char name[MAX_NAME_LENGTH];
    uint32_t uuid[3];
    uint8_t FW_Version[4];
    uint8_t RF_Version[4];
    uint8_t HW_Version[4];

    bool active;

    hid_device *_dev_handle;

    float current[3];
    float frequency[3];
    float voltage[3];
    float boost_Voltage;

    uint32_t lastTimeSeenMs;
};
