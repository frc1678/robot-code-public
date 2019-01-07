#pragma once

#include "ctre/phoenix/cci/CCI.h"
#include <stdint.h>
#include "ctre/phoenix/ErrorCode.h"
#include "ctre/phoenix/autocache.h"

extern "C" {
    CCIEXPORT int32_t c_SetCANInterface(const char * canBusInterface);
    CCIEXPORT ctre::phoenix::ErrorCode c_DestroyAll();
    CCIEXPORT ctre::phoenix::ErrorCode c_StartAll();
    CCIEXPORT void c_Autocache_SetAutocacheLevel(ctre::phoenix::AutocacheState state);
}
