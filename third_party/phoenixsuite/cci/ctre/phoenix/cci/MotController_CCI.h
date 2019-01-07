#include "ctre/phoenix/cci/CCI.h"
#include "ctre/phoenix/ErrorCode.h"
#include "ctre/phoenix/motorcontrol/ControlMode.h"
#include <cstddef>

extern "C"{
	CCIEXPORT void* c_MotController_Create1(int baseArbId);
    CCIEXPORT void c_MotController_DestroyAll();
    CCIEXPORT ctre::phoenix::ErrorCode c_MotController_Destroy(void *handle);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetDeviceNumber(void *handle, int *deviceNumber);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetDescription(void *handle, char * toFill, int toFillByteSz, size_t * numBytesFilled);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_SetDemand(void *handle, int mode, int demand0, int demand1);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_Set_4(void *handle, int mode, double demand0, double demand1, int demand1Type);
	CCIEXPORT void c_MotController_SetNeutralMode(void *handle, int neutralMode);
	CCIEXPORT void c_MotController_SetSensorPhase(void *handle, bool PhaseSensor);
	CCIEXPORT void c_MotController_SetInverted(void *handle, bool invert);
	CCIEXPORT void c_MotController_SetInverted_2(void *handle, int invertType);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigFactoryDefault(void *handle, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigOpenLoopRamp(void *handle, double secondsFromNeutralToFull, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigClosedLoopRamp(void *handle, double secondsFromNeutralToFull, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigPeakOutputForward(void *handle, double percentOut, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigPeakOutputReverse(void *handle, double percentOut, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigNominalOutputForward(void *handle, double percentOut, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigNominalOutputReverse(void *handle, double percentOut, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigNeutralDeadband(void *handle, double percentDeadband, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigVoltageCompSaturation(void *handle, double voltage, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigVoltageMeasurementFilter(void *handle, int filterWindowSamples, int timeoutMs);
	CCIEXPORT void c_MotController_EnableVoltageCompensation(void *handle, bool enable);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetInverted(void *handle, bool *invert);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetBusVoltage(void *handle, double *voltage);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetMotorOutputPercent(void *handle, double *percentOutput);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetOutputCurrent(void *handle, double *current);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetTemperature(void *handle, double *temperature);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigSelectedFeedbackSensor(void *handle, int feedbackDevice, int pidIdx, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigSelectedFeedbackCoefficient(void *handle, double coefficient, int pidIdx, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigRemoteFeedbackFilter(void *handle, int deviceID, int remoteSensorSource, int remoteOrdinal, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigSensorTerm(void *handle, int sensorTerm, int feedbackDevice, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetSelectedSensorPosition(void *handle, int *param, int pidIdx);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetSelectedSensorVelocity(void *handle, int *param, int pidIdx);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_SetSelectedSensorPosition(void *handle, int sensorPos, int pidIdx,int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_SetControlFramePeriod(void *handle, int frame, int periodMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_SetStatusFramePeriod(void *handle, int frame, uint8_t periodMs, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetStatusFramePeriod(void *handle, int frame, int *periodMs, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigVelocityMeasurementPeriod(void *handle, int period, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigVelocityMeasurementWindow(void *handle, int windowSize, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigForwardLimitSwitchSource(void *handle, int type, int normalOpenOrClose, int deviceID, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigReverseLimitSwitchSource(void *handle, int type, int normalOpenOrClose, int deviceID, int timeoutMs);
	CCIEXPORT void c_MotController_OverrideLimitSwitchesEnable(void *handle, bool enable);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigForwardSoftLimitThreshold(void *handle, int forwardSensorLimit, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigReverseSoftLimitThreshold(void *handle, int reverseSensorLimit, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigForwardSoftLimitEnable(void *handle, bool enable, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigReverseSoftLimitEnable(void *handle, bool enable, int timeoutMs);
	CCIEXPORT void c_MotController_OverrideSoftLimitsEnable(void *handle, bool enable);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_Config_kP(void *handle, int slotIdx, double value, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_Config_kI(void *handle, int slotIdx, double value, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_Config_kD(void *handle, int slotIdx, double value, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_Config_kF(void *handle, int slotIdx, double value, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_Config_IntegralZone(void *handle, int slotIdx, double izone, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigAllowableClosedloopError(void *handle, int slotIdx, int allowableClosedLoopError, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigMaxIntegralAccumulator(void *handle, int slotIdx, double iaccum, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigClosedLoopPeakOutput(void *handle, int slotIdx, double percentOut, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigClosedLoopPeriod(void *handle, int slotIdx, int loopTimeMs, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_SetIntegralAccumulator(void *handle, double iaccum, int pidIdx, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetClosedLoopError(void *handle, int *closedLoopError, int pidIdx);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetIntegralAccumulator(void *handle, double *iaccum, int pidIdx);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetErrorDerivative(void *handle, double *derror, int pidIdx);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_SelectProfileSlot(void *handle, int slotIdx, int pidIdx);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryPosition(void *handle, int *param);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryVelocity(void *handle, int *param);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryHeading(void *handle, double *param);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryPosition_3(void *handle, int *param, int pidIdx);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryVelocity_3(void *handle, int *param, int pidIdx);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryArbFeedFwd_3(void *handle, double *param, int pidIdx);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryAll(void *handle, int * vel, int * pos, double *heading);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetActiveTrajectoryAll_5(void *handle, int * vel, int * pos, double *arbFeedFwd, int pidIdx);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigMotionCruiseVelocity(void *handle, int sensorUnitsPer100ms, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigMotionAcceleration(void *handle, int sensorUnitsPer100msPerSec, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ClearMotionProfileTrajectories(void *handle);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetMotionProfileTopLevelBufferCount(void *handle, int * value);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_PushMotionProfileTrajectory(void *handle, double position,
			double velocity, double headingDeg, int profileSlotSelect, bool isLastPoint, bool zeroPos);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_PushMotionProfileTrajectory_2(
		void *handle, double position, double velocity, double headingDeg,
		int profileSlotSelect0, int profileSlotSelect1, bool isLastPoint, bool zeroPos, int durationMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_PushMotionProfileTrajectory_3(void *handle, double position, double velocity, double arbFeedFwd, double auxiliaryPos, double auxiliaryVel, double auxiliaryArbFeedFwd, uint32_t profileSlotSelect0, uint32_t profileSlotSelect1, bool isLastPoint, bool zeroPos0, uint32_t timeDur, bool useAuxPID);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_StartMotionProfile(void *handle, void * streamHandle, uint32_t minBufferedPts, ctre::phoenix::motorcontrol::ControlMode controlMode);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_IsMotionProfileFinished(void *handle, bool * value);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_IsMotionProfileTopLevelBufferFull(void *handle, bool * value);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ProcessMotionProfileBuffer(void *handle);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetMotionProfileStatus(void *handle,
			size_t *topBufferRem, size_t *topBufferCnt, int *btmBufferCnt,
			bool *hasUnderrun, bool *isUnderrun, bool *activePointValid,
			bool *isLast, int *profileSlotSelect, int *outputEnable);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetMotionProfileStatus_2(void *handle,
		    size_t *topBufferRem, size_t *topBufferCnt, int *btmBufferCnt,
			bool *hasUnderrun, bool *isUnderrun, bool *activePointValid,
			bool *isLast, int *profileSlotSelect, int *outputEnable, int *timeDurMs,
			int *profileSlotSelect1);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ClearMotionProfileHasUnderrun(void *handle,
			int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ChangeMotionControlFramePeriod(void *handle,
			int periodMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigMotionProfileTrajectoryPeriod(
			void *handle, int durationMs, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigMotionProfileTrajectoryInterpolationEnable(void *handle, bool enable, int timeoutMs);
    CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigFeedbackNotContinuous(void *handle,
            bool feedbackNotContinuous, int timeoutMs);
    CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigRemoteSensorClosedLoopDisableNeutralOnLOS(void *handle,
            bool remoteSensorClosedLoopDisableNeutralOnLOS, int timeoutMs);
    CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigClearPositionOnLimitF(void *handle,
            bool clearPositionOnLimitF, int timeoutMs);
    CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigClearPositionOnLimitR(void *handle,
            bool clearPositionOnLimitR, int timeoutMs);
    CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigClearPositionOnQuadIdx(void *handle,
            bool clearPositionOnQuadIdx, int timeoutMs);
    CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigLimitSwitchDisableNeutralOnLOS(void *handle,
            bool limitSwitchDisableNeutralOnLOS, int timeoutMs);
    CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigSoftLimitDisableNeutralOnLOS(void *handle,
            bool softLimitDisableNeutralOnLOS, int timeoutMs);
    CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigPulseWidthPeriod_EdgesPerRot(void *handle,
            int pulseWidthPeriod_EdgesPerRot, int timeoutMs);
    CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigPulseWidthPeriod_FilterWindowSz(void *handle,
            int pulseWidthPeriod_FilterWindowSz, int timeoutMs);
    CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetLastError(void *handle);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetFirmwareVersion(void *handle, int *);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_HasResetOccurred(void *handle,bool *);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigSetCustomParam(void *handle, int newValue, int paramIndex, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigGetCustomParam(void *handle, int *readValue, int paramIndex, int timoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigSetParameter(void *handle, int param, double value, uint8_t subValue, int ordinal, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigGetParameter(void *handle, int param, double *value, int ordinal, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigGetParameter_6(void *handle, int32_t param, int32_t valueToSend, int32_t * valueRecieved, uint8_t * subValue, int32_t ordinal, int32_t timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigPeakCurrentLimit(void *handle, int amps, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigPeakCurrentDuration(void *handle, int milliseconds, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ConfigContinuousCurrentLimit(void *handle, int amps, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_EnableCurrentLimit(void *handle, bool enable);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_SetLastError(void *handle, int error);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetAnalogIn(void *handle, int * param);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_SetAnalogPosition(void *handle,int newPosition, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetAnalogInRaw(void *handle, int * param);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetAnalogInVel(void *handle, int * param);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetQuadraturePosition(void *handle, int * param);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_SetQuadraturePosition(void *handle,int newPosition, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetQuadratureVelocity(void *handle, int * param);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetPulseWidthPosition(void *handle, int * param);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_SetPulseWidthPosition(void *handle,int newPosition, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetPulseWidthVelocity(void *handle, int * param);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetPulseWidthRiseToFallUs(void *handle, int * param);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetPulseWidthRiseToRiseUs(void *handle, int * param);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetPinStateQuadA(void *handle, int * param);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetPinStateQuadB(void *handle, int * param);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetPinStateQuadIdx(void *handle, int * param);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_IsFwdLimitSwitchClosed(void *handle, int * param);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_IsRevLimitSwitchClosed(void *handle, int * param);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetFaults(void *handle, int * param);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetStickyFaults(void *handle, int * param);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_ClearStickyFaults(void *handle, int timeoutMs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_SelectDemandType(void *handle, bool enable);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_SetMPEOutput(void *handle, int MpeOutput);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_EnableHeadingHold(void *handle, bool enable);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetAnalogInAll(void *handle, int * withOv, int * raw, int * vel);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetQuadratureSensor(void *handle, int * pos, int * vel);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetPulseWidthAll(void *handle, int * pos, int * vel, int * riseToRiseUs, int * riseToFallUs);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetQuadPinStates(void *handle, int * quadA, int * quadB, int * quadIdx);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetLimitSwitchState(void *handle, int * isFwdClosed, int * isRevClosed);
	CCIEXPORT ctre::phoenix::ErrorCode c_MotController_GetClosedLoopTarget(void *handle, int * value, int pidIdx);
}
