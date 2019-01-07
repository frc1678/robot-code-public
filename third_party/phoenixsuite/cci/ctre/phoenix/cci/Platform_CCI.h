#pragma once

#include "ctre/phoenix/cci/CCI.h"
#include "ctre/phoenix/platform/Platform.h"
#include <stdint.h>

using namespace ctre::phoenix::platform;

extern "C" {
    CCIEXPORT int32_t c_SimCreate(DeviceType type, int id);
    CCIEXPORT int32_t c_SimDestroy(DeviceType type, int id);
    CCIEXPORT int32_t c_SimDestroyAll();
    CCIEXPORT int32_t c_SimConfigGet(DeviceType type, uint32_t param, uint32_t valueToSend, uint32_t & outValueReceived, uint32_t & outSubvalue, uint32_t ordinal, int id);
    CCIEXPORT int32_t c_SimConfigSet(DeviceType type, uint32_t param, uint32_t value, uint32_t subValue, uint32_t ordinal, int id);
}
