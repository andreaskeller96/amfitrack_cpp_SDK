//-----------------------------------------------------------------------------
//
//                              AMFITECH APS
//
//                          ALL RIGHTS RESERVED
//
//-----------------------------------------------------------------------------

#pragma once

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
        void reset_devices();

        bool get_sensor(uint8_t device_id, AMFITRACK_Sensor *sensor) const;
        bool set_sensor(AMFITRACK_Sensor const *sensor);
        bool reset_sensor(uint8_t device_id);

        bool get_source(uint8_t device_id, AMFITRACK_Source *source) const;
        bool set_source(AMFITRACK_Source const *source);
        bool reset_source(uint8_t device_id);

        static bool is_valid_device_id(uint8_t device_id);
        static std::size_t device_count();

    private:
        AMFITRACK_NEW();
        ~AMFITRACK_NEW() = default;

        AMFITRACK_NEW(AMFITRACK_NEW const &) = delete;
        AMFITRACK_NEW &operator=(AMFITRACK_NEW const &) = delete;

        void setup_device_slots();

#ifdef USE_THREAD_BASED
        mutable std::mutex _mutex;
#endif

        std::array<AMFITRACK_Sensor, AMFITRACK_NEW_DEVICE_COUNT> _sensors;
        std::array<AMFITRACK_Source, AMFITRACK_NEW_DEVICE_COUNT> _sources;

};

#endif
/** @} */ // end of module
