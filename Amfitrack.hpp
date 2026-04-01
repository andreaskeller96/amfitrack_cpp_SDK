//-----------------------------------------------------------------------------
//                              AMFITECH APS
//                          ALL RIGHTS RESERVED
//
// $URL: $
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
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "project_conf.h"
#include "lib_AmfiProt_API.hpp"
#ifdef USE_THREAD_BASED
#include <mutex>
#endif // USE_THREAD_BASED

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------
#define MAX_NAME_LENGTH 64
#define MAX_NUMBER_OF_DEVICES 254

//-----------------------------------------------------------------------------
// Type declarations
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Variables and constants
//-----------------------------------------------------------------------------
class AMFITRACK
{
public:
    static AMFITRACK &getInstance()
    {
        static AMFITRACK instance;
        return instance;
    }

    /* Call first to initialize USB and connect to devices */
    void initialize_amfitrack();

    // TODO(pkyle): Using 'start_amfitrack_task' workflow causes data races when calling
    //  getDevicePose, getSensorMeasurement(), etc... For now our team is NOT using this workflow
    //  because we believe it is dangerous. Instead, we 'manually' call 'amfitrack_main_loop()' then
    //  'getDevicePose()' in series.
    /* Starts the main thread, that reads data from all connected devices */
    void start_amfitrack_task(void);

    /* Stops the main thread */
    void stop_amfitrack_task(void);

    void amfitrack_main_loop(void);

    void setDeviceName(uint8_t DeviceID, char *name, uint8_t length);

    void setConfiguration(uint8_t DeviceID, uint32_t UID, lib_Generic_Parameter_Value_t parameter);

    void setDeviceActive(uint8_t DeviceID);
    /* Return if a device is connected */
    bool getDeviceActive(uint8_t DeviceID);

    void setDevicePose(uint8_t DeviceID, lib_AmfiProt_Amfitrack_Pose_t Pose);
    /* Get the pose for a specific device */
    void getDevicePose(uint8_t DeviceID, lib_AmfiProt_Amfitrack_Pose_t *Pose);

    void setDeviceIMU(uint8_t DeviceID, lib_AmfiProt_Amfitrack_IMU_t imuData);
    void getDeviceIMU(uint8_t DeviceID, lib_AmfiProt_Amfitrack_IMU_t *imuData);

    void setSensorMeasurements(uint8_t DeviceID, lib_AmfiProt_Amfitrack_Sensor_Measurement_t SensorMeasurement);

    void getSensorMeasurements(uint8_t DeviceID, lib_AmfiProt_Amfitrack_Sensor_Measurement_t *SensorMeasurement);

#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
    void setSensorTimestamp(uint8_t DeviceID, std::chrono::steady_clock::time_point time_stamp);

    void getSensorTimestamp(uint8_t DeviceID, std::chrono::steady_clock::time_point *time_stamp);
#endif

private:
#ifdef USE_THREAD_BASED
    std::mutex mutName;                                // Protects array of names
#endif                                                 // USE_THREAD_BASED
    char Name[MAX_NUMBER_OF_DEVICES][MAX_NAME_LENGTH]; // Array of character arrays to store device names

#ifdef USE_THREAD_BASED
    std::mutex mutDeviceActive; // Protects array of DeviceActive bools
#endif                          // USE_THREAD_BASED
    bool DeviceActive[MAX_NUMBER_OF_DEVICES];

#ifdef USE_ACTIVE_DEVICE_HANDLING
#ifdef USE_THREAD_BASED
    std::mutex mutDeviceLastSeen; // Protects array of last seen times
#endif                            // USE_THREAD_BASED
    time_t DeviceLastTimeSeen[MAX_NUMBER_OF_DEVICES];
#endif

#ifdef USE_THREAD_BASED
    std::mutex mutPosition; // Protects array of position poses
#endif                      // USE_THREAD_BASED
    lib_AmfiProt_Amfitrack_Pose_t Position[MAX_NUMBER_OF_DEVICES];

#ifdef USE_THREAD_BASED
    std::mutex mutIMU; // Protects array of position poses
#endif                 // USE_THREAD_BASED
    lib_AmfiProt_Amfitrack_IMU_t IMUData[MAX_NUMBER_OF_DEVICES];

#ifdef USE_THREAD_BASED
    std::mutex mutSensorMeasurements; // Protects array of sensor measurement structs
#endif                                // USE_THREAD_BASED
    lib_AmfiProt_Amfitrack_Sensor_Measurement_t SensorMeasurements[MAX_NUMBER_OF_DEVICES];

    static void background_amfitrack_task(AMFITRACK *);
    bool checkDeviceDisconnected(uint8_t DeviceID);

    AMFITRACK();
    ~AMFITRACK();
#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
#ifdef USE_THREAD_BASED
    std::mutex mutSensorTimestamps; // Protects array of sensor timestamps
#endif                              // USE_THREAD_BASED
    std::chrono::steady_clock::time_point SensorTimestamps[MAX_NUMBER_OF_DEVICES];
#endif
};

