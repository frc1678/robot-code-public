/*
 * �Software License Agreement
 *
 *�Copyright (C) Cross The Road Electronics.� All rights
 *�reserved.
 *�
 *�Cross The Road Electronics (CTRE) licenses to you the right to�
 *�use, publish, and distribute copies of CRF (Cross The Road) firmware files (*.crf) and Software
 * API Libraries ONLY when in use with Cross The Road Electronics hardware products.
 *�
 *�THE SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT
 *�WARRANTY OF ANY KIND, EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT
 *�LIMITATION, ANY WARRANTY OF MERCHANTABILITY, FITNESS FOR A
 *�PARTICULAR PURPOSE, TITLE AND NON-INFRINGEMENT. IN NO EVENT SHALL
 *�CROSS THE ROAD ELECTRONICS BE LIABLE FOR ANY INCIDENTAL, SPECIAL,�
 *�INDIRECT OR CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF
 *�PROCUREMENT OF SUBSTITUTE GOODS, TECHNOLOGY OR SERVICES, ANY CLAIMS
 *�BY THIRD PARTIES (INCLUDING BUT NOT LIMITED TO ANY DEFENSE
 *�THEREOF), ANY CLAIMS FOR INDEMNITY OR CONTRIBUTION, OR OTHER
 *�SIMILAR COSTS, WHETHER ASSERTED ON THE BASIS OF CONTRACT, TORT
 *�(INCLUDING NEGLIGENCE), BREACH OF WARRANTY, OR OTHERWISE
 */
 
 #pragma once
 
#include "ctre/phoenix/cci/CCI.h"
#include "ctre/phoenix/ErrorCode.h"
#include <map>
#include <cstddef>
 
 static std::map<void *, bool> pigeonPresent;
 
 extern "C"{
	CCIEXPORT void *c_PigeonIMU_Create2(int talonDeviceID);
	CCIEXPORT void *c_PigeonIMU_Create1(int deviceNumber);
    CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_Destroy(void *handle);
    CCIEXPORT void c_PigeonIMU_DestroyAll();
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetDescription(void *handle, char * toFill, int toFillByteSz, size_t * numBytesFilled);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_ConfigSetParameter(void *handle, int param, double value, uint8_t subValue, int ordinal, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_ConfigGetParameter(void *handle, int param, double *value, int ordinal, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_ConfigGetParameter_6(void *handle, int32_t param, int32_t valueToSend, int32_t * valueRecieved, uint8_t * subValue, int32_t ordinal, int32_t timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_ConfigSetCustomParam(void *handle, int newValue, int paramIndex, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_ConfigGetCustomParam(void *handle, int *readValue, int paramIndex, int timoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_ConfigFactoryDefault(void *handle, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_SetYaw(void *handle, double angleDeg, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_AddYaw(void *handle, double angleDeg, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_SetYawToCompass(void *handle, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_SetFusedHeading(void *handle, double angleDeg, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_AddFusedHeading(void *handle, double angleDeg, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_SetFusedHeadingToCompass(void *handle, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_SetAccumZAngle(void *handle, double angleDeg, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_SetTemperatureCompensationDisable(void *handle, int bTempCompDisable, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_SetCompassDeclination(void *handle, double angleDegOffset, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_SetCompassAngle(void *handle, double angleDeg, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_EnterCalibrationMode(void *handle, int calMode, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetGeneralStatus(void *handle, int *state, int *currentMode, int *calibrationError, int *bCalIsBooting, double *tempC, int *upTimeSec, int *noMotionBiasCount, int *tempCompensationCount, int *lastError);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetLastError(void *handle);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_Get6dQuaternion(void *handle, double wxyz[4]);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetYawPitchRoll(void *handle, double ypr[3]);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetAccumGyro(void *handle, double xyz_deg[3]);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetAbsoluteCompassHeading(void *handle, double *value);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetCompassHeading(void *handle, double *value);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetCompassFieldStrength(void *handle, double *value);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetTemp(void *handle, double *value);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetState(void *handle, int *state);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetUpTime(void *handle, int *value);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetRawMagnetometer(void *handle, short rm_xyz[3]);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetBiasedMagnetometer(void *handle, short bm_xyz[3]);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetBiasedAccelerometer(void *handle, short ba_xyz[3]);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetRawGyro(void *handle, double xyz_dps[3]);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetAccelerometerAngles(void *handle, double tiltAngles[3]);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetFusedHeading2(void *handle, int *bIsFusing, int *bIsValid, double *value, int *lastError);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetFusedHeading1(void *handle, double *value);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetResetCount(void *handle, int *value);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetResetFlags(void *handle, int *value);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetFirmwareVersion(void *handle, int * firmwareVers);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_HasResetOccurred(void *handle, bool * hasReset);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_SetLastError(void *handle, int value);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetFaults(void *handle, int * param);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetStickyFaults(void *handle, int * param);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_ClearStickyFaults(void *handle, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_SetStatusFramePeriod(void *handle, int frame, uint8_t periodMs, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_GetStatusFramePeriod(void *handle, int frame, int *periodMs, int timeoutMs) ;
	CCIEXPORT ctre::phoenix::ErrorCode c_PigeonIMU_SetControlFramePeriod(void *handle, int frame, int periodMs) ;
}
