//-----------------------------------------------------------------------------
//
//                              AMFITECH APS
//
//                          ALL RIGHTS RESERVED
//
//-----------------------------------------------------------------------------

#pragma once

#include "AmfitrackDeviceTypes.h"
#ifdef __cplusplus

//-----------------------------------------------------------------------------
// Section: Includes
//-----------------------------------------------------------------------------

#include "src/Amfitrack_Devices.h"

#include <array>
#include <cstddef>
#include <cstdint>

#ifdef USE_THREAD_BASED
#include <mutex>
#endif

//-----------------------------------------------------------------------------
// Section: Define
//-----------------------------------------------------------------------------

#define AMFITRACK_NEW_DEVICE_COUNT 255U
#define AMFITRACK_NEW_BROADCAST_DEVICE_ID 255U

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
// Section: Class
//-----------------------------------------------------------------------------

class AMFITRACK_NEW
{
    public:
        static AMFITRACK_NEW &getInstance();

        void init();
        void start_task();
        void stop_task();
        void run();
        void reset_devices();

        bool get_sensor(uint8_t device_id, AMFITRACK_Sensor *sensor) const;
        bool reset_sensor(uint8_t device_id);
        // Set for sensor
        bool set(uint8_t device_id, Pose_t const &pose);
        bool set(uint8_t device_id, IMU_t const &imu);
#if defined(_WIN32) || defined(__linux__) || defined(__APPLE__)
        bool set(uint8_t device_id, std::chrono::steady_clock::time_point timestamp);
#endif


        bool get_source(uint8_t device_id, AMFITRACK_Source *source) const;
        bool reset_source(uint8_t device_id);
        // Set for source
        bool set(uint8_t device_id, Current_t const &current);
        bool set(uint8_t device_id, Frequency_t const &frequency);
        bool set(uint8_t device_id, Voltage_t const &voltage);
        bool set(uint8_t device_id, Calibration_t const &calibration);

        static bool is_valid_device_id(uint8_t device_id);
        static std::size_t device_count();

    private:
        AMFITRACK_NEW();
        ~AMFITRACK_NEW() = default;

        AMFITRACK_NEW(AMFITRACK_NEW const &) = delete;
        AMFITRACK_NEW &operator=(AMFITRACK_NEW const &) = delete;

        static void background_amfitrack_task(AMFITRACK_NEW *);
        void setup_device_slots();
        uint32_t get_time_ms();

#ifdef USE_THREAD_BASED
        mutable std::mutex _mutex;
#endif

        std::array<AMFITRACK_Sensor, AMFITRACK_NEW_DEVICE_COUNT> _sensors;
        std::array<AMFITRACK_Source, AMFITRACK_NEW_DEVICE_COUNT> _sources;

};

#endif
/** @} */ // end of module
