#pragma once

#include "ctre/phoenix/cci/CCI.h"
#include "ctre/phoenix/ErrorCode.h"
#include <set>
#include <cstddef>

namespace CANifier_CCI{
	enum GeneralPin{
	QUAD_IDX = 0,
	QUAD_B = 1,
	QUAD_A = 2,
	LIMR = 3,
	LIMF = 4,
	SDA = 5,
	SCL = 6,
	SPI_CS = 7,
	SPI_MISO_PWM2P = 8,
	SPI_MOSI_PWM1P = 9,
	SPI_CLK_PWM0P = 10,
	};
}

extern "C"{
	CCIEXPORT void *c_CANifier_Create1(int deviceNumber);
    CCIEXPORT void c_CANifier_DestroyAll();
    CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_Destroy(void *handle);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_GetDescription(void *handle, char * toFill, int toFillByteSz, size_t * numBytesFilled);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_SetLEDOutput(void *handle,  uint32_t  dutyCycle,  uint32_t  ledChannel);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_SetGeneralOutputs(void *handle,  uint32_t  outputsBits,  uint32_t  isOutputBits);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_SetGeneralOutput(void *handle,  uint32_t  outputPin,  bool  outputValue,  bool  outputEnable);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_SetPWMOutput(void *handle,  uint32_t  pwmChannel,  uint32_t  dutyCycle);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_EnablePWMOutput(void *handle, uint32_t pwmChannel, bool bEnable);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_GetGeneralInputs(void *handle, bool allPins[], uint32_t capacity);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_GetGeneralInput(void *handle, uint32_t inputPin, bool * measuredInput);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_GetPWMInput(void *handle,  uint32_t  pwmChannel,  double dutyCycleAndPeriod [2]);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_GetLastError(void *handle);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_GetBusVoltage(void *handle, double * batteryVoltage);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_GetQuadraturePosition(void *handle, int * pos);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_SetQuadraturePosition(void *handle, int pos, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_GetQuadratureVelocity(void *handle, int * vel);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_GetQuadratureSensor(void *handle, int * pos, int * vel);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_ConfigVelocityMeasurementPeriod(void *handle, int period, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_ConfigVelocityMeasurementWindow(void *handle, int window, int timeoutMs);
    CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_ConfigClearPositionOnLimitF(void *handle,
            bool clearPositionOnLimitF, int timeoutMs);
    CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_ConfigClearPositionOnLimitR(void *handle,
            bool clearPositionOnLimitR, int timeoutMs);
    CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_ConfigClearPositionOnQuadIdx(void *handle,
            bool clearPositionOnQuadIdx, int timeoutMs);
	CCIEXPORT void c_CANifier_SetLastError(void *handle, int error);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_ConfigSetParameter(void *handle, int param, double value, uint8_t subValue, int ordinal, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_ConfigGetParameter(void *handle, int param, double *value, int ordinal, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_ConfigGetParameter_6(void *handle, int32_t param, int32_t valueToSend, int32_t * valueRecieved, uint8_t * subValue, int32_t ordinal, int32_t timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_ConfigSetCustomParam(void *handle, int newValue, int paramIndex, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_ConfigGetCustomParam(void *handle, int *readValue, int paramIndex, int timoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_ConfigFactoryDefault(void *handle, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_GetFaults(void *handle, int * param) ;
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_GetStickyFaults(void *handle, int * param) ;
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_ClearStickyFaults(void *handle, int timeoutMs) ;
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_GetFirmwareVersion(void *handle, int *firmwareVers);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_HasResetOccurred(void *handle, bool * hasReset);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_SetStatusFramePeriod(void *handle, int frame, uint8_t periodMs, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_GetStatusFramePeriod(void *handle, int frame, int *periodMs, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_CANifier_SetControlFramePeriod(void *handle, int frame,	int periodMs) ;
}
