//-----------------------------------------------------------------------------
//                              AMFITECH APS
//                          ALL RIGHTS RESERVED
//
// $URL: $
// $Rev: $
// $Date: $
// $Author: $
//
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Includes
//-----------------------------------------------------------------------------
#include "lib_AmfiProt_API.hpp"

using namespace AMFITRACK_API_LIB;


//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Type declarations
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Function prototypes
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Variables and constants
//-----------------------------------------------------------------------------

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------
AmfiProt_API::AmfiProt_API()
{
    libQueue_Init(&incomingBulkPointer, sizeof(incomingBulkData) / sizeof(incomingBulkData[0]));
    libQueue_Init(&outgoingBulkPointer, sizeof(outgoingBulkData) / sizeof(outgoingBulkData[0]));
}


AmfiProt_API::~AmfiProt_API()
{

}

void AmfiProt_API::isRequestAckSet(uint8_t idx)
{
    // To check if its requesting an ack (Can it be done a better way?)
    lib_AmfiProt_Frame_t amfiFrame;

    memcpy(&amfiFrame, &this->outgoingBulkData[idx], sizeof(amfiFrame));
    uint8_t controlBits = (amfiFrame.header.packetType & lib_AmfiProt_packetType_Mask);
    if (controlBits)
    {
        this->isTransmitting = true;
        time(&_retransmitTimer);
    }
    else
    {
        libQueue_Remove(&(this->outgoingBulkPointer));
        this->_retransmitCount = 0;
    }
}

void AmfiProt_API::process_incoming_queue(void)
{
    while (!libQueue_Empty(&(this->incomingBulkPointer)))
    {
        size_t idx = libQueue_Read(&(this->incomingBulkPointer));
        this->lib_AmfiProt_ProcessFrame(NULL, &(this->incomingBulkData[idx]), NULL);
        libQueue_Remove(&(this->incomingBulkPointer));
    }
}

void AmfiProt_API::clear_isTransmitting(lib_AmfiProt_Frame_t* frame)
{
#ifdef USB_CONNECTION_DEBUG_INFO
    printf("clear_isTransmitting: frame package number: %u | last package number: %u \n", frame->header.packetNumber, this->_apiHandle->packetNumber[frame->header.source]);
#endif
    if (frame->header.packetNumber == this->packetNumber[frame->header.source])
    {
        if (!libQueue_Empty(&(this->outgoingBulkPointer)))
        {
            libQueue_Remove(&(this->outgoingBulkPointer));
        }
        this->_retransmitCount = 0;
        this->isTransmitting = false;
    }
}

bool AmfiProt_API::queue_frame(void const* payload, uint8_t length, uint8_t payloadType, lib_AmfiProt_packetType_t packetType, uint8_t destination)
{
    static uint8_t packageNumber = 0;

    bool isOk = false;
    lib_AmfiProt_Frame_t amfiFrame;

    packetNumber[destination] = packageNumber;
    if (this->lib_AmfiProt_EncodeFrame(&amfiFrame, payload, length, payloadType, packetNumber[destination], destination, packetType))
    {
        if (!libQueue_Full(&this->outgoingBulkPointer))
        {
            uint8_t frameLength = this->lib_AmfiProt_FrameSize(&amfiFrame);
            memcpy(&(this->outgoingBulkData[libQueue_Write(&this->outgoingBulkPointer)]), &(amfiFrame), frameLength);
            libQueue_Add(&this->outgoingBulkPointer);
            isOk = true;
            packageNumber++;
        }
    }

    return isOk;
}

void AmfiProt_API::amfiprot_run(void)
{
    this->process_incoming_queue();

    static time_t current_timer;
    time(&current_timer);
    double diffTime = difftime(current_timer, _retransmitTimer);

    if (this->isTransmitting && (diffTime >= 1.0))
    {
#ifdef USB_CONNECTION_DEBUG_INFO
        printf("Transmit failed, retransmitting package \n");
#endif
        this->_retransmitCount++;
        if (this->_retransmitCount >= 3)
        {
            this->_retransmitCount = 0;
            libQueue_Remove(&(this->outgoingBulkPointer));
        }
        this->isTransmitting = false;
    }
}

void AmfiProt_API::libAmfiProt_handle_Ack(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
#ifdef USB_CONNECTION_DEBUG_INFO
    printf("TxUID: %u | Ack \n", frame->header.source);
#endif
    this->clear_isTransmitting(frame);
}

void AmfiProt_API::libAmfiProt_handle_ReplySuccess(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
#ifdef USB_CONNECTION_DEBUG_INFO
    printf("TxUID: %u | Success reply\n", frame->header.source);
#endif  
}

void AmfiProt_API::libAmfiProt_handle_ReplyFailure(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
#ifdef USB_CONNECTION_DEBUG_INFO
    printf("TxUID: %u | Failure reply\n", frame->header.source);
#endif
}

void AmfiProt_API::libAmfiProt_ReplyInvalid(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
#ifdef USB_CONNECTION_DEBUG_INFO
    printf("TxUID: %u | Invalid reply: %u", frame->header.source, frame->header.payloadType);
#endif
}

void AmfiProt_API::libAmfiProt_handle_ReplyNotImplemented(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
#ifdef USB_CONNECTION_DEBUG_INFO
    printf("TxUID: %u | Not implemented reply: %u", frame->header.source, frame->header.payloadType);
#endif
}

void AmfiProt_API::libAmfiProt_handle_ReplyInvalidRequest(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
#ifdef USB_CONNECTION_DEBUG_INFO
    printf("TxUID: %u | Invalid request: %u", frame->header.source, frame->header.payloadType);
#endif
}

void AmfiProt_API::libAmfiProt_handle_AlternativeProcessing(void* handle, lib_AmfiProt_Frame_t* frame, void* routing_handle)
{
    this->lib_AmfiProt_Amfitrack_processFrame(handle, frame, routing_handle);
}