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

#ifndef LIB_AMFIPROT_API_HPP_
#define LIB_AMFIPROT_API_HPP_

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include <string.h>
#include <cstdlib>
#include <iostream>
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <string.h>
#include <vector>
#include <memory>
#include "lib_AmfiProt.hpp"
#include "lib_AmfiProt_Amfitrack.hpp"

#ifdef __cplusplus
extern "C"
{
#endif
#include "../lib_queue/lib_Queue.h"
#include "../lib_crc/lib_CRC.h"
#ifdef __cplusplus
}
#endif // __cplusplus

namespace AMFITRACK_API_LIB
{
    //-----------------------------------------------------------------------------
    // Defines
    //-----------------------------------------------------------------------------
    constexpr auto VID = 0x0C17;
    constexpr auto PID_Source = 0x0D01; // Source
    constexpr auto PID_Sensor = 0x0D12; // Sensor

    //-----------------------------------------------------------------------------
    // Type declarations
    //-----------------------------------------------------------------------------

    //-----------------------------------------------------------------------------
    // Functions
    //-----------------------------------------------------------------------------
    class AmfiProt_API : public lib_AmfiProt, public  lib_AmfiProt_AmfiTrack
    {
    public:
        static AmfiProt_API& getInstance()
        {
            static AmfiProt_API instance;
            return instance;
        }

        bool isTransmitting;
        lib_AmfiProt_Frame_t outgoingBulkData[10];
        libQueue_Pointer_t outgoingBulkPointer;
        lib_AmfiProt_Frame_t incomingBulkData[10];
        libQueue_Pointer_t incomingBulkPointer;

        /* Must always run! */
        void amfiprot_run(void);

        bool queue_frame(void const* payload, uint8_t length, uint8_t payloadType, lib_AmfiProt_packetType_t packetType, uint8_t destination);

        void isRequestAckSet(uint8_t idx);
        void clear_isTransmitting(lib_AmfiProt_Frame_t* frame);

        void libAmfiProt_handle_RequestProcedureSpec(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_ReplyProcedureSpec(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_RequestProcedureCall(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_ReplyProcedureCall(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_RequestDeviceID(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_RespondDeviceID(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_SetTxID(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_RequestFirmwareVersion(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_FirmwareVersion(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_FirmwareStart(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_FirmwareData(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_FirmwareEnd(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_RequestDeviceName(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_ReplyDeviceName(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_RequestConfigurationValue(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_ReplyConfigurationValue(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_SetConfigurationValue(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_RequestConfigurationName(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_ReplyConfigurationName(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_LoadDefault(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_SaveAsDefault(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_RequestConfigurationNameAndUID(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_ConfigurationNameAndUID(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_RequestConfigurationValueUID(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_ConfigurationValueUID(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_SetConfigurationValueUID(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_RequestConfigurationCategory(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_ConfigurationCategory(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_RequestConfigurationValueCount(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_ConfigurationValueCount(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_RequestCategoryCount(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_CategoryCount(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_Reboot(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_DebugOutput(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_ResetParameter(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_RequestFirmwareVersionPerID(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_AlternativeProcessing(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;

        void libAmfiProt_ReplyInvalid(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;

        void libAmfiProt_handle_Ack(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_ReplySuccess(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_ReplyNotImplemented(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_ReplyFailure(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void libAmfiProt_handle_ReplyInvalidRequest(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;

        void lib_AmfiProt_Amfitrack_handle_SourceCalibration(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void lib_AmfiProt_Amfitrack_handle_SourceMeasurement(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void lib_AmfiProt_Amfitrack_handle_SensorMeasurement(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void lib_AmfiProt_Amfitrack_handle_RawBfield(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void lib_AmfiProt_Amfitrack_handle_NormalizedBfield(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void lib_AmfiProt_Amfitrack_handle_BfieldPhase(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void lib_AmfiProt_Amfitrack_handle_NormalizedBfieldImu(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void lib_AmfiProt_Amfitrack_handle_SignData(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void lib_AmfiProt_Amfitrack_handle_PllData(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void lib_AmfiProt_Amfitrack_handle_RawFloats(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void lib_AmfiProt_Amfitrack_handle_SetPhaseModulation(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void lib_AmfiProt_Amfitrack_handle_SourceCoilCalData(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
        void lib_AmfiProt_Amfitrack_handle_AlternativeProcessing(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle) override;
    private:
        AmfiProt_API();
        ~AmfiProt_API();

        void process_incoming_queue(void);

        uint8_t packetNumber[255];

        uint8_t _retransmitCount;
        bool _lastPackageNumberError;
        time_t _retransmitTimer;
    };
};
//-----------------------------------------------------------------------------
// Variables and constants
//-----------------------------------------------------------------------------



#endif //LIB_AMFIPROT_API_HPP_