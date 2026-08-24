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
#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>
#include "lib_AmfiProt.hpp"
#include "lib_AmfiProt_Amfitrack.hpp"
#include "lib_fifo.hpp"

#include <atomic>
#include <chrono>

#ifdef USE_THREAD_BASED
#include <iostream>
#include <thread>
#endif

#include "lib_CRC.hpp"

//-----------------------------------------------------------------------------
// Defines
//-----------------------------------------------------------------------------
#define MAX_PAYLOAD_SIZE 54
//-----------------------------------------------------------------------------
// Type declarations
//-----------------------------------------------------------------------------

// What last happened to a frame exchanged with one device. Packed into a single
// word so a thread other than the one running the protocol can read it without a
// lock; count makes a repeat of the same packet number a new event.
struct lib_AmfiProt_FrameEvent
{
	uint16_t count;		  // 0: nothing recorded yet
	uint8_t packetNumber; // the packet this concerns
	uint8_t payloadType;  // replies only, 0 otherwise
};

//-----------------------------------------------------------------------------
// Functions
//-----------------------------------------------------------------------------
class AmfiProt_API : public lib_AmfiProt, public lib_AmfiProt_AmfiTrack
{
  public:
	static AmfiProt_API &getInstance()
	{
		static AmfiProt_API instance;
		return instance;
	}

	bool isTransmitting;

	// Indexed by TxID: the last frame written to that device, the last transport
	// ack it sent and the last reply it sent, each carrying the packet number it
	// concerns. A sender whose frames go NoAck - firmware data does - has no
	// transport ack to wait on and correlates on these instead.
	std::atomic<uint32_t> lastSentFrame[256]{};
	std::atomic<uint32_t> lastAckFrame[256]{};
	std::atomic<uint32_t> lastReplyFrame[256]{};

	static lib_AmfiProt_FrameEvent frame_event(std::atomic<uint32_t> const &slot);

	lib_fifo<lib_AmfiProt_Frame_t, 50> outgoingBulk_FiFo;
	lib_fifo<lib_AmfiProt_Frame_t, 50> incomingBulk_FiFo;

	/* Must always run! */
	void amfiprot_run(void);

	// packetNumberInOut reports the number the frame was queued with. With
	// reusePacketNumber it is instead the number to send: a retransmit carrying
	// the number the device already saw rather than a fresh one.
	bool queue_frame(void const *payload, uint8_t length, uint8_t payloadType, lib_AmfiProt_packetType_t packetType, uint8_t destination,
					 uint8_t *packetNumberInOut = nullptr, bool reusePacketNumber = false);

	bool deserialize_frame(void const *pData, uint8_t length);

	bool isDataReadyForTransmit(size_t *QueueDataLength, uint8_t *TxID, void **TransmitData);
	void set_transmit_ongoing_and_check_respons_request(bool removeFromQueue);

	void isRequestAckSet(bool removeFromQueue);
	void clear_isTransmitting(lib_AmfiProt_Frame_t *frame);

	void libAmfiProt_handle_RequestProcedureSpec(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_ReplyProcedureSpec(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_RequestProcedureCall(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_ReplyProcedureCall(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_RequestDeviceID(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_RespondDeviceID(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_SetTxID(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_RequestFirmwareVersion(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_FirmwareVersion(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_FirmwareStart(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_FirmwareData(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_FirmwareEnd(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_RequestDeviceName(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_ReplyDeviceName(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_RequestConfigurationValue(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_ReplyConfigurationValue(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_SetConfigurationValue(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_RequestConfigurationName(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_ReplyConfigurationName(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_LoadDefault(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_SaveAsDefault(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_RequestConfigurationNameAndUID(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_ConfigurationNameAndUID(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_RequestConfigurationValueUID(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_ConfigurationValueUID(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_SetConfigurationValueUID(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_RequestConfigurationCategory(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_ConfigurationCategory(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_RequestConfigurationValueCount(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_ConfigurationValueCount(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_RequestCategoryCount(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_CategoryCount(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_Reboot(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_DebugOutput(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_ResetParameter(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_RequestFirmwareVersionPerID(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_ReplyFirmwareVersionPerID(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_AlternativeProcessing(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;

	void libAmfiProt_ReplyInvalid(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;

	void libAmfiProt_handle_Ack(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_ReplySuccess(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_ReplyNotImplemented(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_ReplyFailure(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void libAmfiProt_handle_ReplyInvalidRequest(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;

	void lib_AmfiProt_Amfitrack_handle_SourceCalibration(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void lib_AmfiProt_Amfitrack_handle_SourceMeasurement(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void lib_AmfiProt_Amfitrack_handle_SensorMeasurement(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void lib_AmfiProt_Amfitrack_handle_SensorStatus(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void lib_AmfiProt_Amfitrack_handle_RawBfield(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void lib_AmfiProt_Amfitrack_handle_NormalizedBfield(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void lib_AmfiProt_Amfitrack_handle_BfieldPhase(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void lib_AmfiProt_Amfitrack_handle_NormalizedBfieldImu(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void lib_AmfiProt_Amfitrack_handle_SignData(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void lib_AmfiProt_Amfitrack_handle_PllData(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void lib_AmfiProt_Amfitrack_handle_RawFloats(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void lib_AmfiProt_Amfitrack_handle_SetPhaseModulation(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void lib_AmfiProt_Amfitrack_handle_SourceCoilCalData(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;
	void lib_AmfiProt_Amfitrack_handle_AlternativeProcessing(void *handle, lib_AmfiProt_Frame_t *frame, void *routing_handle) override;

  private:
	AmfiProt_API();
	~AmfiProt_API();

	void process_incoming_queue(void);

	static void record_frame_event(std::atomic<uint32_t> &slot, uint8_t packetNumber, uint8_t payloadType);

	uint8_t packetNumber[256];

	// Shared by every destination, and read-modify-written from both producers.
	std::atomic<uint8_t> _packetNumberCounter{0};

	uint8_t _retransmitCount;
	bool _lastPackageNumberError;
	// Use chrono for sub-second accuracy instead of time_t
	std::chrono::steady_clock::time_point _retransmitTimer;
	std::chrono::steady_clock::time_point _queueFullLogTime;
};

//-----------------------------------------------------------------------------
// Variables and constants
//-----------------------------------------------------------------------------

#endif // LIB_AMFIPROT_API_HPP_
