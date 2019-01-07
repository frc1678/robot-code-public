#pragma once

#include "ctre/phoenix/cci/CCI.h"

extern "C" {
    CCIEXPORT void c_FeedEnable(int timeoutMs);
    CCIEXPORT bool c_GetEnableState();
	CCIEXPORT void c_SetTransmitEnable(bool en);
	CCIEXPORT bool c_GetTransmitEnable();
    CCIEXPORT int c_GetPhoenixVersion();
}
