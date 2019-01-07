#pragma once

#include "ctre/phoenix/cci/CCI.h"
#include "ctre/phoenix/ErrorCode.h"
#include <set>
#include <cstddef>

extern "C"{
	CCIEXPORT void *c_BuffTrajPointStream_Create1();
    CCIEXPORT void c_BuffTrajPointStream_DestroyAll();
    CCIEXPORT ctre::phoenix::ErrorCode c_BuffTrajPointStream_Destroy(void *handle);
	CCIEXPORT ctre::phoenix::ErrorCode c_BuffTrajPointStream_Clear(void *handle);
	CCIEXPORT ctre::phoenix::ErrorCode c_BuffTrajPointStream_Write(void *handle, double position, double velocity, double arbFeedFwd, double auxiliaryPos, double auxiliaryVel, double auxiliaryArbFeedFwd, uint32_t profileSlotSelect0, uint32_t profileSlotSelect1, bool isLastPoint, bool zeroPos, uint32_t timeDur, bool useAuxPID);
	CCIEXPORT ctre::phoenix::ErrorCode c_BuffTrajPointStream_Lookup(void *handle, void ** outObject);
}
