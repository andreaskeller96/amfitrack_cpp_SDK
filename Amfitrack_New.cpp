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

#include "Amfitrack_New.h"

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

AMFITRACK_NEW &AMFITRACK_NEW::getInstance()
{
    static AMFITRACK_NEW instance;
    return instance;
}

AMFITRACK_NEW::AMFITRACK_NEW()
{
    setup_device_slots();
}

void AMFITRACK_NEW::init()
{
#ifdef USE_THREAD_BASED
    const std::lock_guard<std::mutex> lock(_mutex);
#endif

    setup_device_slots();
}

void AMFITRACK_NEW::reset_devices()
{
#ifdef USE_THREAD_BASED
    const std::lock_guard<std::mutex> lock(_mutex);
#endif

    setup_device_slots();
}

bool AMFITRACK_NEW::get_sensor(uint8_t device_id, AMFITRACK_Sensor *sensor) const
{
    if ((sensor == nullptr) || !is_valid_device_id(device_id))
    {
        return false;
    }

#ifdef USE_THREAD_BASED
    const std::lock_guard<std::mutex> lock(_mutex);
#endif

    *sensor = _sensors[device_id];
    return true;
}

bool AMFITRACK_NEW::set_sensor(AMFITRACK_Sensor const *sensor)
{
    if ((sensor == nullptr) || !is_valid_device_id(sensor->deviceId))
    {
        return false;
    }

#ifdef USE_THREAD_BASED
    const std::lock_guard<std::mutex> lock(_mutex);
#endif

    _sensors[sensor->deviceId] = *sensor;
    return true;
}

bool AMFITRACK_NEW::reset_sensor(uint8_t device_id)
{
    if (!is_valid_device_id(device_id))
    {
        return false;
    }

#ifdef USE_THREAD_BASED
    const std::lock_guard<std::mutex> lock(_mutex);
#endif

    _sensors[device_id].reset();
    _sensors[device_id].deviceId = device_id;
    return true;
}

bool AMFITRACK_NEW::get_source(uint8_t device_id, AMFITRACK_Source *source) const
{
    if ((source == nullptr) || !is_valid_device_id(device_id))
    {
        return false;
    }

#ifdef USE_THREAD_BASED
    const std::lock_guard<std::mutex> lock(_mutex);
#endif

    *source = _sources[device_id];
    return true;
}

bool AMFITRACK_NEW::set_source(AMFITRACK_Source const *source)
{
    if ((source == nullptr) || !is_valid_device_id(source->deviceId))
    {
        return false;
    }

#ifdef USE_THREAD_BASED
    const std::lock_guard<std::mutex> lock(_mutex);
#endif

    _sources[source->deviceId] = *source;
    return true;
}

bool AMFITRACK_NEW::reset_source(uint8_t device_id)
{
    if (!is_valid_device_id(device_id))
    {
        return false;
    }

#ifdef USE_THREAD_BASED
    const std::lock_guard<std::mutex> lock(_mutex);
#endif

    _sources[device_id].reset();
    _sources[device_id].deviceId = device_id;
    return true;
}

bool AMFITRACK_NEW::is_valid_device_id(uint8_t device_id)
{
    return (device_id < AMFITRACK_NEW_DEVICE_COUNT) &&
           (device_id != AMFITRACK_NEW_BROADCAST_DEVICE_ID);
}

std::size_t AMFITRACK_NEW::device_count()
{
    return AMFITRACK_NEW_DEVICE_COUNT;
}

void AMFITRACK_NEW::setup_device_slots()
{
    for (std::size_t device_id = 0; device_id < AMFITRACK_NEW_DEVICE_COUNT; device_id++)
    {
        _sensors[device_id].reset();
        _sensors[device_id].deviceId = static_cast<uint8_t>(device_id);

        _sources[device_id].reset();
        _sources[device_id].deviceId = static_cast<uint8_t>(device_id);
    }
}

