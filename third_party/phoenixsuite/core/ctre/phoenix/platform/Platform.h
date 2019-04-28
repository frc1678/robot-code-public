#pragma once
#include "ctre/phoenix/platform/Platform-pack.h"
#include <stdint.h>
#include <string>

/* small wrinkle for RIO platform */
#ifdef __FRC_ROBORIO__
	struct tCANStreamMessage;
#endif

namespace ctre {
namespace phoenix {
namespace platform {
namespace can {
	/**
	 * "plain old data" container for holding a CAN Frame Event.
	 * Assignment of this type resolves to a copy-by-value.
	 */
	typedef struct _canframe_t {
		uint32_t arbID; //!< ArbID of the CAN frame.
		uint32_t timeStampUs; //!< Timestamp if receive event.  Zero otherwise.
		uint8_t data[8]; //!< Data bytes
		uint32_t flags; //!< Zero for now.  Can be used for detecting arbID type (29bit vs 11bit).
		uint8_t dlc; //!< Number of bytes in payload
	} canframe_t;

	//-------------- Low Level CANBus interface, this is required if using phoenix-canutil--------------------------//
	void CANbus_GetStatus(float *busUtilPerc, uint32_t *busOffCount, uint32_t *txFullCount, uint32_t *rec, uint32_t *tec, int32_t *status);
	int32_t CANbus_SendFrame(uint32_t messageID, const uint8_t *data, uint8_t dataSize);
	/* assumed blocking */
	int32_t CANbus_ReceiveFrame(canframe_t * toFill, uint32_t frameCap, uint32_t *numFilled);
    
	/**
	 * Set the CAN interface to use, for example on Linux you may select "can0".
	 * @param CANInterface CAN interface string.
	 * @return errorcode, zero if successful.
	 */
    int32_t SetCANInterface(const char * CANInterface);

	//-------------- Mid Level CANBus interface, this is required if NOT using phoenix-canutil, --------------------------//
	void CANComm_SendMessage(uint32_t messageID, const uint8_t *data, uint8_t dataSize, int32_t periodMs, int32_t *status);
	void CANComm_ReceiveMessage(uint32_t *messageID, uint32_t messageIDMask, uint8_t *data, uint8_t *dataSize, uint32_t *timeStamp, int32_t *status);
	void CANComm_OpenStreamSession(uint32_t *sessionHandle, uint32_t messageID, uint32_t messageIDMask, uint32_t maxMessages, int32_t *status);
	void CANComm_CloseStreamSession(uint32_t sessionHandle);
#ifdef __FRC_ROBORIO__
	void CANComm_ReadStreamSession(uint32_t sessionHandle, struct tCANStreamMessage *messages, uint32_t messagesToRead, uint32_t *messagesRead, int32_t *status);
#else
	void CANComm_ReadStreamSession(uint32_t sessionHandle, canframe_t *messages, uint32_t messagesToRead, uint32_t *messagesRead, int32_t *status);
#endif
	int32_t CANComm_GetTxSchedulerStatus(void *unusedControlWorld); // used to be GetControlWord

} //namespace can
} //namespace platform
} //namespace phoenix
} //namespace ctre

namespace ctre {
namespace phoenix {
namespace platform {

    enum DeviceType {TalonSRXType, VictorSPXType, CANifierType, PigeonIMUType};

	/**
	 * @param timeUs	How long to yield current thread in microseconds (us).  
	 *					If platform cannot honor us resolution, round to nearest
	 *					value that platform can honor.
	 */
	void SleepUs(int timeUs);

	/**
	 * Get a stack trace, ignoring the first "offset" symbols.
	 *
	 * @param offset The number of symbols at the top of the stack to ignore
	 */
	std::string GetStackTrace(int offset);

	void ReportError(int isError, int32_t errorCode, int isLVCode,
		const char *details, const char *location, const char *callStack);

    int32_t SimCreate(DeviceType type, int id);

    int32_t SimConfigGet(DeviceType type, uint32_t param, uint32_t valueToSend, uint32_t & outValueReceived, uint32_t & outSubvalue, uint32_t ordinal, uint32_t id);
    
    int32_t SimConfigSet(DeviceType type, uint32_t param, uint32_t value, uint32_t subValue, uint32_t ordinal, uint32_t id);

	int32_t SimDestroy(DeviceType type, int id);
	int32_t SimDestroyAll();

    int32_t DisposePlatform();
    int32_t StartPlatform();

    int32_t DisposeMgr();
    int32_t StartMgr();
    
} // namespace platform
} // namespace phoenix
} // namespace ctre
