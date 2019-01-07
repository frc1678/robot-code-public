#pragma once

#include "ctre/phoenix/ErrorCode.h"
#include "ctre/phoenix/paramEnum.h"
#include "ctre/phoenix/core/GadgeteerUartClient.h"
#include "ctre/phoenix/motorcontrol/IMotorController.h"
#include "ctre/phoenix/motorcontrol/ControlMode.h"
#include "ctre/phoenix/motorcontrol/DemandType.h"
#include "ctre/phoenix/motorcontrol/Faults.h"
#include "ctre/phoenix/motorcontrol/FollowerType.h"
#include "ctre/phoenix/motorcontrol/InvertType.h"
#include "ctre/phoenix/motorcontrol/StickyFaults.h"
#include "ctre/phoenix/motorcontrol/VelocityMeasPeriod.h"
#include "ctre/phoenix/motion/TrajectoryPoint.h"
#include "ctre/phoenix/motion/MotionProfileStatus.h"
#include "ctre/phoenix/motion/BufferedTrajectoryPointStream.h"
#include "ctre/phoenix/CANBusAddressable.h"
#include "ctre/phoenix/CustomParamConfiguration.h"

#include <string>

/* forward proto's */
namespace ctre {
namespace phoenix {
namespace motorcontrol {
namespace lowlevel {
class MotControllerWithBuffer_LowLevel;
class MotController_LowLevel;
}
}
}
}

namespace ctre {
namespace phoenix {
namespace motorcontrol {
namespace can {

struct BasePIDSetConfiguration {

	double selectedFeedbackCoefficient;

	BasePIDSetConfiguration() :
		selectedFeedbackCoefficient(1.0)
	{
	}

	std::string toString() {
		return toString("");
	}

    std::string toString(const std::string &prependString) {
        return prependString + ".selectedFeedbackCoefficient = " + std::to_string(selectedFeedbackCoefficient) + ";\n";
    
    }
};// struct BasePIDSetConfiguration
struct FilterConfiguration {

	int remoteSensorDeviceID; 
	RemoteSensorSource remoteSensorSource;  

    FilterConfiguration() :
        remoteSensorDeviceID(0), 
        remoteSensorSource(RemoteSensorSource::RemoteSensorSource_Off) 	
    {
    }

	std::string toString() {
		return toString("");
	}

    std::string toString(std::string prependString) {
        std::string retstr = prependString + ".remoteSensorDeviceID = " + std::to_string(remoteSensorDeviceID) + ";\n";
        retstr += prependString + ".remoteSensorSource = " + RemoteSensorSourceRoutines::toString(remoteSensorSource) + ";\n";
        return retstr;
    }
	

}; // struct FilterConfiguration
struct FilterConfigUtil {
	private:
		static FilterConfiguration _default;
	public:
		static bool RemoteSensorDeviceIDDifferent (const FilterConfiguration & settings) { return (!(settings.remoteSensorDeviceID == _default.remoteSensorDeviceID)); }
		static bool RemoteSensorSourceDifferent (const FilterConfiguration & settings) { return (!(settings.remoteSensorSource == _default.remoteSensorSource)); }
		static bool FilterConfigurationDifferent (const FilterConfiguration & settings) { return RemoteSensorDeviceIDDifferent(settings) || RemoteSensorSourceDifferent(settings); }
};
struct SlotConfiguration{

	double kP; 
	double kI; 
	double kD; 
	double kF; 
	int integralZone; 
	int allowableClosedloopError; 
	double maxIntegralAccumulator; 
	double closedLoopPeakOutput;
	int closedLoopPeriod;
		
	SlotConfiguration() : 
		kP(0.0), 
		kI(0.0),
		kD(0.0),
		kF(0.0),
		integralZone(0), 
		allowableClosedloopError(0), 
		maxIntegralAccumulator(0.0),
		closedLoopPeakOutput(1.0),
		closedLoopPeriod(1)
	{
	}

	std::string toString() {
		return toString("");
	}

    std::string toString(std::string prependString) {

        std::string retstr = prependString + ".kP = " + std::to_string(kP) + ";\n"; 
        retstr += prependString + ".kI = " + std::to_string(kI) + ";\n"; 
        retstr += prependString + ".kD = " + std::to_string(kD) + ";\n"; 
        retstr += prependString + ".kF = " + std::to_string(kF) + ";\n"; 
        retstr += prependString + ".integralZone = " + std::to_string(integralZone) + ";\n"; 
        retstr += prependString + ".allowableClosedloopError = " + std::to_string(allowableClosedloopError) + ";\n"; 
        retstr += prependString + ".maxIntegralAccumulator = " + std::to_string(maxIntegralAccumulator) + ";\n"; 
        retstr += prependString + ".closedLoopPeakOutput = " + std::to_string(closedLoopPeakOutput) + ";\n";
        retstr += prependString + ".closedLoopPeriod = " + std::to_string(closedLoopPeriod) + ";\n";

        return retstr;

    }
    
};// struct BaseSlotConfiguration

class SlotConfigUtil {
	private:
		static struct SlotConfiguration _default;
	public:
		static bool KPDifferent (const SlotConfiguration & settings) { return (!(settings.kP == _default.kP)); }
		static bool KIDifferent (const SlotConfiguration & settings) { return (!(settings.kI == _default.kI)); }
		static bool KDDifferent (const SlotConfiguration & settings) { return (!(settings.kD == _default.kD)); }
		static bool KFDifferent (const SlotConfiguration & settings) { return (!(settings.kF == _default.kF)); }
		static bool IntegralZoneDifferent (const SlotConfiguration & settings) { return (!(settings.integralZone == _default.integralZone)); }
		static bool AllowableClosedloopErrorDifferent (const SlotConfiguration & settings) { return (!(settings.allowableClosedloopError == _default.allowableClosedloopError)); }
		static bool MaxIntegralAccumulatorDifferent (const SlotConfiguration & settings) { return (!(settings.maxIntegralAccumulator == _default.maxIntegralAccumulator)); }
		static bool ClosedLoopPeakOutputDifferent (const SlotConfiguration & settings) { return (!(settings.closedLoopPeakOutput == _default.closedLoopPeakOutput)); }
		static bool ClosedLoopPeriodDifferent (const SlotConfiguration & settings) { return (!(settings.closedLoopPeriod == _default.closedLoopPeriod)); }
};


struct BaseMotorControllerConfiguration : ctre::phoenix::CustomParamConfiguration {
	double openloopRamp;
	double closedloopRamp; 
	double peakOutputForward;
	double peakOutputReverse;
	double nominalOutputForward; 
	double nominalOutputReverse; 
	double neutralDeadband;
	double voltageCompSaturation; 
	int voltageMeasurementFilter;
    VelocityMeasPeriod velocityMeasurementPeriod; 
	int velocityMeasurementWindow; 
	int forwardSoftLimitThreshold; 
	int reverseSoftLimitThreshold; 
	bool forwardSoftLimitEnable; 
	bool reverseSoftLimitEnable; 
	SlotConfiguration slot0;
	SlotConfiguration slot1;
	SlotConfiguration slot2;
	SlotConfiguration slot3;
	bool auxPIDPolarity; 
	FilterConfiguration remoteFilter0;
	FilterConfiguration remoteFilter1;
    int motionCruiseVelocity; 
	int motionAcceleration; 
	int motionProfileTrajectoryPeriod; 
    bool feedbackNotContinuous;
    bool remoteSensorClosedLoopDisableNeutralOnLOS;
    bool clearPositionOnLimitF;
    bool clearPositionOnLimitR;
    bool clearPositionOnQuadIdx;
    bool limitSwitchDisableNeutralOnLOS;
    bool softLimitDisableNeutralOnLOS;
    int pulseWidthPeriod_EdgesPerRot;
    int pulseWidthPeriod_FilterWindowSz;
	bool trajectoryInterpolationEnable;

    BaseMotorControllerConfiguration() :
        openloopRamp(0.0),
        closedloopRamp(0.0),
        peakOutputForward(1.0),
        peakOutputReverse(-1.0),
        nominalOutputForward(0.0),
        nominalOutputReverse(0.0),
        neutralDeadband(41.0 / 1023.0),
        voltageCompSaturation(0.0),
        voltageMeasurementFilter(32),
        velocityMeasurementPeriod(Period_100Ms),
        velocityMeasurementWindow(64),
        forwardSoftLimitThreshold(0),
        reverseSoftLimitThreshold(0), 
        forwardSoftLimitEnable(false),
        reverseSoftLimitEnable(false),
        auxPIDPolarity(false), 
        motionCruiseVelocity(0),
        motionAcceleration(0),
        motionProfileTrajectoryPeriod(0),
        feedbackNotContinuous(false),
        remoteSensorClosedLoopDisableNeutralOnLOS(false),
        clearPositionOnLimitF(false),
        clearPositionOnLimitR(false),
        clearPositionOnQuadIdx(false),
        limitSwitchDisableNeutralOnLOS(false),
        softLimitDisableNeutralOnLOS(false),
        pulseWidthPeriod_EdgesPerRot(1),
        pulseWidthPeriod_FilterWindowSz(1),
		trajectoryInterpolationEnable(true)

	{
	}

	std::string toString() {
		return toString("");
	}

    std::string toString(std::string prependString) {

        std::string retstr = prependString + ".openloopRamp = " + std::to_string(openloopRamp) + ";\n";
        retstr += prependString + ".closedloopRamp = " + std::to_string(closedloopRamp) + ";\n"; 
        retstr += prependString + ".peakOutputForward = " + std::to_string(peakOutputForward) + ";\n";
        retstr += prependString + ".peakOutputReverse = " + std::to_string(peakOutputReverse) + ";\n";
        retstr += prependString + ".nominalOutputForward = " + std::to_string(nominalOutputForward) + ";\n"; 
        retstr += prependString + ".nominalOutputReverse = " + std::to_string(nominalOutputReverse) + ";\n"; 
        retstr += prependString + ".neutralDeadband = " + std::to_string(neutralDeadband) + ";\n";
        retstr += prependString + ".voltageCompSaturation = " + std::to_string(voltageCompSaturation) + ";\n"; 
        retstr += prependString + ".voltageMeasurementFilter = " + std::to_string(voltageMeasurementFilter) + ";\n";
        retstr += prependString + ".velocityMeasurementPeriod = " + VelocityMeasPeriodRoutines::toString(velocityMeasurementPeriod) + ";\n"; 
        retstr += prependString + ".velocityMeasurementWindow = " + std::to_string(velocityMeasurementWindow) + ";\n"; 
        retstr += prependString + ".forwardSoftLimitThreshold = " + std::to_string(forwardSoftLimitThreshold) + ";\n"; 
        retstr += prependString + ".reverseSoftLimitThreshold = " + std::to_string(reverseSoftLimitThreshold) + ";\n"; 
        retstr += prependString + ".forwardSoftLimitEnable = " + std::to_string(forwardSoftLimitEnable) + ";\n"; 
        retstr += prependString + ".reverseSoftLimitEnable = " + std::to_string(reverseSoftLimitEnable) + ";\n"; 
        retstr += slot0.toString(prependString + ".slot0");
        retstr += slot1.toString(prependString + ".slot1");
        retstr += slot2.toString(prependString + ".slot2");
        retstr += slot3.toString(prependString + ".slot3");
        retstr += prependString + ".auxPIDPolarity = " + std::to_string(auxPIDPolarity) + ";\n"; 
        retstr += remoteFilter0.toString(prependString + ".remoteFilter0");
        retstr += remoteFilter1.toString(prependString + ".remoteFilter1");
        retstr += prependString + ".motionCruiseVelocity = " + std::to_string(motionCruiseVelocity) + ";\n"; 
        retstr += prependString + ".motionAcceleration = " + std::to_string(motionAcceleration) + ";\n"; 
        retstr += prependString + ".motionProfileTrajectoryPeriod = " + std::to_string(motionProfileTrajectoryPeriod) + ";\n"; 
        retstr += prependString + ".feedbackNotContinuous = " + std::to_string(feedbackNotContinuous) + ";\n";
        retstr += prependString + ".remoteSensorClosedLoopDisableNeutralOnLOS = " + std::to_string(remoteSensorClosedLoopDisableNeutralOnLOS) + ";\n";
        retstr += prependString + ".clearPositionOnLimitF = " + std::to_string(clearPositionOnLimitF) + ";\n";
        retstr += prependString + ".clearPositionOnLimitR = " + std::to_string(clearPositionOnLimitR) + ";\n";
        retstr += prependString + ".clearPositionOnQuadIdx = " + std::to_string(clearPositionOnQuadIdx) + ";\n";
        retstr += prependString + ".limitSwitchDisableNeutralOnLOS = " + std::to_string(limitSwitchDisableNeutralOnLOS) + ";\n";
        retstr += prependString + ".softLimitDisableNeutralOnLOS = " + std::to_string(softLimitDisableNeutralOnLOS) + ";\n";
        retstr += prependString + ".pulseWidthPeriod_EdgesPerRot = " + std::to_string(pulseWidthPeriod_EdgesPerRot) + ";\n";
        retstr += prependString + ".pulseWidthPeriod_FilterWindowSz = " + std::to_string(pulseWidthPeriod_FilterWindowSz) + ";\n";
		retstr += prependString + ".trajectoryInterpolationEnable = " + std::to_string(trajectoryInterpolationEnable) + ";\n";

        retstr += CustomParamConfiguration::toString(prependString);

        return retstr;
    }
    
    
};// struct BaseMotorControllerConfiguration

class BaseMotorControllerUtil : public ctre::phoenix::CustomParamConfigUtil {
    private :
        static struct BaseMotorControllerConfiguration _default;
    public:
        static bool OpenloopRampDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.openloopRamp == _default.openloopRamp)) || !settings.enableOptimizations; }
        static bool ClosedloopRampDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.closedloopRamp == _default.closedloopRamp)) || !settings.enableOptimizations; }
        static bool PeakOutputForwardDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.peakOutputForward == _default.peakOutputForward)) || !settings.enableOptimizations; }
        static bool PeakOutputReverseDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.peakOutputReverse == _default.peakOutputReverse)) || !settings.enableOptimizations; }
        static bool NominalOutputForwardDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.nominalOutputForward == _default.nominalOutputForward)) || !settings.enableOptimizations; }
        static bool NominalOutputReverseDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.nominalOutputReverse == _default.nominalOutputReverse)) || !settings.enableOptimizations; }
        static bool NeutralDeadbandDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.neutralDeadband == _default.neutralDeadband)) || !settings.enableOptimizations; }
        static bool VoltageCompSaturationDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.voltageCompSaturation == _default.voltageCompSaturation)) || !settings.enableOptimizations; }
        static bool VoltageMeasurementFilterDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.voltageMeasurementFilter == _default.voltageMeasurementFilter)) || !settings.enableOptimizations; }
        static bool VelocityMeasurementPeriodDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.velocityMeasurementPeriod == _default.velocityMeasurementPeriod)) || !settings.enableOptimizations; }
        static bool VelocityMeasurementWindowDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.velocityMeasurementWindow == _default.velocityMeasurementWindow)) || !settings.enableOptimizations; }
        static bool ForwardSoftLimitThresholdDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.forwardSoftLimitThreshold == _default.forwardSoftLimitThreshold)) || !settings.enableOptimizations; }
        static bool ReverseSoftLimitThresholdDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.reverseSoftLimitThreshold == _default.reverseSoftLimitThreshold)) || !settings.enableOptimizations; }
        static bool ForwardSoftLimitEnableDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.forwardSoftLimitEnable == _default.forwardSoftLimitEnable)) || !settings.enableOptimizations; }
        static bool ReverseSoftLimitEnableDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.reverseSoftLimitEnable == _default.reverseSoftLimitEnable)) || !settings.enableOptimizations; }
        static bool AuxPIDPolarityDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.auxPIDPolarity == _default.auxPIDPolarity)) || !settings.enableOptimizations; }
        static bool MotionCruiseVelocityDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.motionCruiseVelocity == _default.motionCruiseVelocity)) || !settings.enableOptimizations; }
        static bool MotionAccelerationDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.motionAcceleration == _default.motionAcceleration)) || !settings.enableOptimizations; }
        static bool MotionProfileTrajectoryPeriodDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.motionProfileTrajectoryPeriod == _default.motionProfileTrajectoryPeriod)) || !settings.enableOptimizations; }
        static bool FeedbackNotContinuousDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.feedbackNotContinuous == _default.feedbackNotContinuous)) || !settings.enableOptimizations; }
        static bool RemoteSensorClosedLoopDisableNeutralOnLOSDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.remoteSensorClosedLoopDisableNeutralOnLOS == _default.remoteSensorClosedLoopDisableNeutralOnLOS)) || !settings.enableOptimizations; }
        static bool ClearPositionOnLimitFDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.clearPositionOnLimitF == _default.clearPositionOnLimitF)) || !settings.enableOptimizations; }
        static bool ClearPositionOnLimitRDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.clearPositionOnLimitR == _default.clearPositionOnLimitR)) || !settings.enableOptimizations; }
        static bool ClearPositionOnQuadIdxDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.clearPositionOnQuadIdx == _default.clearPositionOnQuadIdx)) || !settings.enableOptimizations; }
        static bool LimitSwitchDisableNeutralOnLOSDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.limitSwitchDisableNeutralOnLOS == _default.limitSwitchDisableNeutralOnLOS)) || !settings.enableOptimizations; }
        static bool SoftLimitDisableNeutralOnLOSDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.softLimitDisableNeutralOnLOS == _default.softLimitDisableNeutralOnLOS)) || !settings.enableOptimizations; }
        static bool PulseWidthPeriod_EdgesPerRotDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.pulseWidthPeriod_EdgesPerRot == _default.pulseWidthPeriod_EdgesPerRot)) || !settings.enableOptimizations; }
        static bool PulseWidthPeriod_FilterWindowSzDifferent (const BaseMotorControllerConfiguration & settings) { return (!(settings.pulseWidthPeriod_FilterWindowSz == _default.pulseWidthPeriod_FilterWindowSz)) || !settings.enableOptimizations; }
		static bool TrajectoryInterpolationEnableDifferent (const BaseMotorControllerConfiguration & settings) {return (!(settings.trajectoryInterpolationEnable == _default.trajectoryInterpolationEnable)) || !settings.enableOptimizations; }
};
/**
 * Base motor controller features for all CTRE CAN motor controllers.
 */
class BaseMotorController: public virtual IMotorController {
private:
	ControlMode m_controlMode = ControlMode::PercentOutput;
	ControlMode m_sendMode = ControlMode::PercentOutput;

	int _arbId = 0;
	double m_setPoint = 0;
	InvertType _invert = InvertType::None;

	ctre::phoenix::ErrorCode ConfigureSlot(const SlotConfiguration &slot, int slotIdx, int timeoutMs, bool enableOptimizations);
	ctre::phoenix::ErrorCode ConfigureFilter(const FilterConfiguration &filter, int ordinal, int timeoutMs, bool enableOptimizations);

protected:
	void* m_handle;
	/**
	 * @return CCI handle for child classes.
	 */
	void* GetHandle();
    /**
     * Configures all base persistant settings.
     *
	 * @param allConfigs        Object with all of the base persistant settings
     * @param timeoutMs
     *              Timeout value in ms. If nonzero, function will wait for
     *              config success and report an error if it times out.
     *              If zero, no blocking or checking is performed.
     *
     * @return Error Code generated by function. 0 indicates no error. 
     */
	virtual ctre::phoenix::ErrorCode BaseConfigAllSettings(const BaseMotorControllerConfiguration &allConfigs, int timeoutMs);
    /**
     * Gets all base persistant settings.
     *
	 * @param allConfigs        Object with all of the base persistant settings
     * @param timeoutMs
     *              Timeout value in ms. If nonzero, function will wait for
     *              config success and report an error if it times out.
     *              If zero, no blocking or checking is performed.
     */
	virtual void BaseGetAllConfigs(BaseMotorControllerConfiguration &allConfigs, int timeoutMs);
    /**
     * Gets all base PID set persistant settings.
     *
	 * @param pid           Object with all of the base PID set persistant settings
     * @param pidIdx        0 for Primary closed-loop. 1 for auxiliary closed-loop.       
     * @param timeoutMs
     *              Timeout value in ms. If nonzero, function will wait for
     *              config success and report an error if it times out.
     *              If zero, no blocking or checking is performed.
     */
	virtual void BaseGetPIDConfigs(BasePIDSetConfiguration &pid, int pidIdx, int timeoutMs);
	
    //------ General Status ----------//
	/**
	 * Gets the output current of the motor controller.
	 *
	 * @return The output current (in amps).
	 */
	virtual double GetOutputCurrent();
public:
	/**
	 * Constructor for motor controllers.
	 *
	 * @param arbId Device ID [0,62]
	 */
	BaseMotorController(int arbId);
	virtual ~BaseMotorController();
	BaseMotorController() = delete;
	BaseMotorController(BaseMotorController const&) = delete;
	BaseMotorController& operator=(BaseMotorController const&) = delete;

    static void DestroyAllMotControllers();

	/**
	 * Returns the Device ID
	 *
	 * @return Device number.
	 */
	virtual int GetDeviceID();
	// ------ Set output routines. ----------//
	/**
	 * Sets the appropriate output on the talon, depending on the mode.
	 * @param mode The output mode to apply.
	 * In PercentOutput, the output is between -1.0 and 1.0, with 0.0 as stopped.
	 * In Current mode, output value is in amperes.
	 * In Velocity mode, output value is in position change / 100ms.
	 * In Position mode, output value is in encoder ticks or an analog value,
	 *   depending on the sensor.
	 * In Follower mode, the output value is the integer device ID of the talon to
	 * duplicate.
	 *
	 * @param value The setpoint value, as described above.
	 *
	 *
	 *	Standard Driving Example:
	 *	_talonLeft.set(ControlMode.PercentOutput, leftJoy);
	 *	_talonRght.set(ControlMode.PercentOutput, rghtJoy);
	 */
	virtual void Set(ControlMode mode, double value);
	/**
     * @deprecated use 4 parameter set
	 * @param mode Sets the appropriate output on the talon, depending on the mode.
	 * @param demand0 The output value to apply.
	 * 	such as advanced feed forward and/or auxiliary close-looping in firmware.
	 * In PercentOutput, the output is between -1.0 and 1.0, with 0.0 as stopped.
	 * In Current mode, output value is in amperes.
	 * In Velocity mode, output value is in position change / 100ms.
	 * In Position mode, output value is in encoder ticks or an analog value,
	 *   depending on the sensor. See
	 * In Follower mode, the output value is the integer device ID of the talon to
	 * duplicate.
	 *
	 * @param demand1 Supplemental value.  This will also be control mode specific for future features.
	 */
	virtual void Set(ControlMode mode, double demand0, double demand1);
	/**
	 * @param mode Sets the appropriate output on the talon, depending on the mode.
	 * @param demand0 The output value to apply.
	 * 	such as advanced feed forward and/or auxiliary close-looping in firmware.
	 * In PercentOutput, the output is between -1.0 and 1.0, with 0.0 as stopped.
	 * In Current mode, output value is in amperes.
	 * In Velocity mode, output value is in position change / 100ms.
	 * In Position mode, output value is in encoder ticks or an analog value,
	 *   depending on the sensor. See
	 * In Follower mode, the output value is the integer device ID of the talon to
	 * duplicate.
	 *
	 * @param demand1Type The demand type for demand1.
	 * Neutral: Ignore demand1 and apply no change to the demand0 output.
	 * AuxPID: Use demand1 to set the target for the auxiliary PID 1.
	 * ArbitraryFeedForward: Use demand1 as an arbitrary additive value to the
	 *	 demand0 output.  In PercentOutput the demand0 output is the motor output,
	 *   and in closed-loop modes the demand0 output is the output of PID0.
	 * @param demand1 Supplmental output value.  Units match the set mode.
	 *
	 *
	 *  Arcade Drive Example:
	 *		_talonLeft.set(ControlMode.PercentOutput, joyForward, DemandType.ArbitraryFeedForward, +joyTurn);
	 *		_talonRght.set(ControlMode.PercentOutput, joyForward, DemandType.ArbitraryFeedForward, -joyTurn);
	 *
	 *	Drive Straight Example:
	 *	Note: Selected Sensor Configuration is necessary for both PID0 and PID1.
	 *		_talonLeft.follow(_talonRght, FollwerType.AuxOutput1);
	 *		_talonRght.set(ControlMode.PercentOutput, joyForward, DemandType.AuxPID, desiredRobotHeading);
	 *
	 *	Drive Straight to a Distance Example:
	 *	Note: Other configurations (sensor selection, PID gains, etc.) need to be set.
	 *		_talonLeft.follow(_talonRght, FollwerType.AuxOutput1);
	 *		_talonRght.set(ControlMode.MotionMagic, targetDistance, DemandType.AuxPID, desiredRobotHeading);
	 */
	virtual void Set(ControlMode mode, double demand0, DemandType demand1Type, double demand1);
	/**
	 * Neutral the motor output by setting control mode to disabled.
	 */
	virtual void NeutralOutput();
	/**
	 * Sets the mode of operation during neutral throttle output.
	 *
	 * @param neutralMode
	 *            The desired mode of operation when the Controller output
	 *            throttle is neutral (ie brake/coast)
	 **/
	virtual void SetNeutralMode(NeutralMode neutralMode);
	/**
	 * Enables a future feature called "Heading Hold".
	 * For now this simply updates the CAN signal to the motor controller.
	 * Future firmware updates will use this.
	 *
	 *	@param enable true/false enable
	 */
	void EnableHeadingHold(bool enable);
	/**
	 * For now this simply updates the CAN signal to the motor controller.
	 * Future firmware updates will use this to control advanced auxiliary loop behavior.
	 *
	 *	@param value
	 */
	void SelectDemandType(bool value);
	//------ Invert behavior ----------//
	/**
	 * Sets the phase of the sensor. Use when controller forward/reverse output
	 * doesn't correlate to appropriate forward/reverse reading of sensor.
	 * Pick a value so that positive PercentOutput yields a positive change in sensor.
	 * After setting this, user can freely call SetInverted() with any value.
	 *
	 * @param PhaseSensor
	 *            Indicates whether to invert the phase of the sensor.
	 */
	virtual void SetSensorPhase(bool PhaseSensor);
	/**
	 * Inverts the hbridge output of the motor controller.
	 *
	 * This does not impact sensor phase and should not be used to correct sensor polarity.
	 *
	 * This will invert the hbridge output but NOT the LEDs.
	 * This ensures....
	 *  - Green LEDs always represents positive request from robot-controller/closed-looping mode.
	 *  - Green LEDs correlates to forward limit switch.
	 *  - Green LEDs correlates to forward soft limit.
	 *
	 * @param invert
	 *            Invert state to set.
	 */
	virtual void SetInverted(bool invert);
	/**
	 * Inverts the hbridge output of the motor controller in relation to the master if present 
	 *
	 * This does not impact sensor phase and should not be used to correct sensor polarity.
	 *
	 * This will allow you to either:
	 *  - Not invert the motor
	 *  - Invert the motor
	 *  - Always follow the master regardless of master's inversion
	 *  - Always oppose the master regardless of master's inversion
	 *
	 * @param invertType
	 *            Invert state to set.
	 */
	virtual void SetInverted(InvertType invertType);
	/**
	 * @return invert setting of motor output.
	 */
	virtual bool GetInverted() const;
	//----- Factory Default Configuration -----//
	/**
	 * Configure all configurations to factory default values
	 * 
	 * @param timeoutMs
	 *            Timeout value in ms. Function will generate error if config is
	 *            not successful within timeout.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigFactoryDefault(int timeoutMs = 50);
	//----- general output shaping ------------------//
	/**
	 * Configures the open-loop ramp rate of throttle output.
	 *
	 * @param secondsFromNeutralToFull
	 *            Minimum desired time to go from neutral to full throttle. A
	 *            value of '0' will disable the ramp.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigOpenloopRamp(double secondsFromNeutralToFull,
			int timeoutMs = 0);
	/**
	 * Configures the closed-loop ramp rate of throttle output.
	 *
	 * @param secondsFromNeutralToFull
	 *            Minimum desired time to go from neutral to full throttle. A
	 *            value of '0' will disable the ramp.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigClosedloopRamp(double secondsFromNeutralToFull,
			int timeoutMs = 0);
	/**
	 * Configures the forward peak output percentage.
	 *
	 * @param percentOut
	 *            Desired peak output percentage. [0,1]
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigPeakOutputForward(double percentOut, int timeoutMs = 0);
	/**
	 * Configures the reverse peak output percentage.
	 *
	 * @param percentOut
	 *            Desired peak output percentage.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigPeakOutputReverse(double percentOut, int timeoutMs = 0);
	/**
	 * Configures the forward nominal output percentage.
	 *
	 * @param percentOut
	 *            Nominal (minimum) percent output. [0,+1]
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigNominalOutputForward(double percentOut,
			int timeoutMs = 0);
	/**
	 * Configures the reverse nominal output percentage.
	 *
	 * @param percentOut
	 *            Nominal (minimum) percent output. [-1,0]
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigNominalOutputReverse(double percentOut,
			int timeoutMs = 0);
	/**
	 * Configures the output deadband percentage.
	 *
	 * @param percentDeadband
	 *            Desired deadband percentage. Minimum is 0.1%, Maximum is 25%.
	 *            Pass 0.04 for 4% (factory default).
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigNeutralDeadband(double percentDeadband,
			int timeoutMs = 0);
	//------ Voltage Compensation ----------//
	/**
	 * Configures the Voltage Compensation saturation voltage.
	 *
	 * @param voltage
	 *            This is the max voltage to apply to the hbridge when voltage
	 *            compensation is enabled.  For example, if 10 (volts) is specified
	 *            and a TalonSRX is commanded to 0.5 (PercentOutput, closed-loop, etc)
	 *            then the TalonSRX will attempt to apply a duty-cycle to produce 5V.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigVoltageCompSaturation(double voltage, int timeoutMs = 0);
	/**
	 * Configures the voltage measurement filter.
	 *
	 * @param filterWindowSamples
	 *            Number of samples in the rolling average of voltage
	 *            measurement.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigVoltageMeasurementFilter(int filterWindowSamples,
			int timeoutMs = 0);
	/**
	 * Enables voltage compensation. If enabled, voltage compensation works in
	 * all control modes.
	 *
	 * @param enable
	 *            Enable state of voltage compensation.
	 **/
	virtual void EnableVoltageCompensation(bool enable);
	//------ General Status ----------//
	/**
	 * Gets the bus voltage seen by the device.
	 *
	 * @return The bus voltage value (in volts).
	 */
	virtual double GetBusVoltage();
	/**
	 * Gets the output percentage of the motor controller.
	 *
	 * @return Output of the motor controller (in percent).
	 */
	virtual double GetMotorOutputPercent();
	/**
	 * @return applied voltage to motor  in volts.
	 */
	virtual double GetMotorOutputVoltage();
	/**
	 * Gets the temperature of the motor controller.
	 *
	 * @return Temperature of the motor controller (in 'C)
	 */
	virtual double GetTemperature();
	//------ sensor selection ----------//
	/**
	 * Select the remote feedback device for the motor controller.
	 * Most CTRE CAN motor controllers will support remote sensors over CAN.
	 *
	 * @param feedbackDevice
	 *            Remote Feedback Device to select.
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigSelectedFeedbackSensor(
			RemoteFeedbackDevice feedbackDevice, int pidIdx = 0, int timeoutMs = 0);
	/**
	 * Select the feedback device for the motor controller.
	 *
	 * @param feedbackDevice
	 *            Feedback Device to select.
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigSelectedFeedbackSensor(
			FeedbackDevice feedbackDevice, int pidIdx = 0, int timeoutMs = 0);
	/**
	 * The Feedback Coefficient is a scalar applied to the value of the
	 * feedback sensor.  Useful when you need to scale your sensor values
	 * within the closed-loop calculations.  Default value is 1.
	 *
	 * Selected Feedback Sensor register in firmware is the decoded sensor value
	 * multiplied by the Feedback Coefficient.
	 *
	 * @param coefficient
	 *            Feedback Coefficient value.  Maximum value of 1.
	 *						Resolution is 1/(2^16).  Cannot be 0.
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigSelectedFeedbackCoefficient(
			double coefficient, int pidIdx = 0, int timeoutMs = 0);
	/**
	 * Select what remote device and signal to assign to Remote Sensor 0 or Remote Sensor 1.
	 * After binding a remote device and signal to Remote Sensor X, you may select Remote Sensor X
	 * as a PID source for closed-loop features.
	 *
	 * @param deviceID
 	 *            The CAN ID of the remote sensor device.
	 * @param remoteSensorSource
	 *            The remote sensor device and signal type to bind.
	 * @param remoteOrdinal
	 *            0 for configuring Remote Sensor 0
	 *            1 for configuring Remote Sensor 1
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigRemoteFeedbackFilter(int deviceID,
			RemoteSensorSource remoteSensorSource, int remoteOrdinal,
			int timeoutMs = 0);
	/**
	 * Select what sensor term should be bound to switch feedback device.
	 * Sensor Sum = Sensor Sum Term 0 - Sensor Sum Term 1
	 * Sensor Difference = Sensor Diff Term 0 - Sensor Diff Term 1
	 * The four terms are specified with this routine.  Then Sensor Sum/Difference
	 * can be selected for closed-looping.
	 *
	 * @param sensorTerm Which sensor term to bind to a feedback source.
	 * @param feedbackDevice The sensor signal to attach to sensorTerm.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigSensorTerm(SensorTerm sensorTerm,
			FeedbackDevice feedbackDevice, int timeoutMs = 0);
	/**
	 * Select what sensor term should be bound to switch feedback device.
	 * Sensor Sum = Sensor Sum Term 0 - Sensor Sum Term 1
	 * Sensor Difference = Sensor Diff Term 0 - Sensor Diff Term 1
	 * The four terms are specified with this routine.  Then Sensor Sum/Difference
	 * can be selected for closed-looping.
	 *
	 * @param sensorTerm Which sensor term to bind to a feedback source.
	 * @param feedbackDevice The sensor signal to attach to sensorTerm.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigSensorTerm(SensorTerm sensorTerm,
			RemoteFeedbackDevice feedbackDevice, int timeoutMs = 0);

	//------- sensor status --------- //
	/**
	 * Get the selected sensor position (in raw sensor units).
	 *
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop. See
	 *            Phoenix-Documentation for how to interpret.
	 *
	 * @return Position of selected sensor (in raw sensor units).
	 */
	virtual int GetSelectedSensorPosition(int pidIdx = 0);
	/**
	 * Get the selected sensor velocity.
	 *
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @return selected sensor (in raw sensor units) per 100ms.
	 * See Phoenix-Documentation for how to interpret.
	 */
	virtual int GetSelectedSensorVelocity(int pidIdx = 0);
	/**
	 * Sets the sensor position to the given value.
	 *
	 * @param sensorPos
	 *            Position to set for the selected sensor (in raw sensor units).
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode SetSelectedSensorPosition(int sensorPos, int pidIdx = 0, int timeoutMs = 50);
	//------ status frame period changes ----------//
	/**
	 * Sets the period of the given control frame.
	 *
	 * @param frame
	 *            Frame whose period is to be changed.
	 * @param periodMs
	 *            Period in ms for the given frame.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode SetControlFramePeriod(ControlFrame frame, int periodMs);
	/**
	 * Sets the period of the given status frame.
	 *
	 * User ensure CAN Bus utilization is not high.
	 *
	 * This setting is not persistent and is lost when device is reset. If this
	 * is a concern, calling application can use HasReset() to determine if the
	 * status frame needs to be reconfigured.
	 *
	 * @param frame
	 *            Frame whose period is to be changed.
	 * @param periodMs
	 *            Period in ms for the given frame.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode SetStatusFramePeriod(StatusFrame frame, uint8_t periodMs,
			int timeoutMs = 0);
	/**
	 * Sets the period of the given status frame.
	 *
	 * User ensure CAN Bus utilization is not high.
	 *
	 * This setting is not persistent and is lost when device is reset. If this
	 * is a concern, calling application can use HasReset() to determine if the
	 * status frame needs to be reconfigured.
	 *
	 * @param frame
	 *            Frame whose period is to be changed.
	 * @param periodMs
	 *            Period in ms for the given frame.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode SetStatusFramePeriod(StatusFrameEnhanced frame,
			uint8_t periodMs, int timeoutMs = 0);
	/**
	 * Gets the period of the given status frame.
	 *
	 * @param frame
	 *            Frame to get the period of.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Period of the given status frame.
	 */
	virtual int GetStatusFramePeriod(StatusFrame frame, int timeoutMs = 0);
	/**
	 * Gets the period of the given status frame.
	 *
	 * @param frame
	 *            Frame to get the period of.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Period of the given status frame.
	 */
	virtual int GetStatusFramePeriod(StatusFrameEnhanced frame, int timeoutMs = 0);
	//----- velocity signal conditionaing ------//
	/**
	 * Sets the period over which velocity measurements are taken.
	 *
	 * @param period
	 *            Desired period for the velocity measurement. @see
	 *            #VelocityMeasPeriod
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigVelocityMeasurementPeriod(VelocityMeasPeriod period,
			int timeoutMs = 0);
	/**
	 * Sets the number of velocity samples used in the rolling average velocity
	 * measurement.
	 *
	 * @param windowSize
	 *            Number of samples in the rolling average of velocity
	 *            measurement. Valid values are 1,2,4,8,16,32. If another value
	 *            is specified, it will truncate to nearest support value.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigVelocityMeasurementWindow(int windowSize,
			int timeoutMs = 0);
	//------ remote limit switch ----------//
	/**
	 * Configures the forward limit switch for a remote source. For example, a
	 * CAN motor controller may need to monitor the Limit-F pin of another Talon
	 * or CANifier.
	 *
	 * @param type
	 *            Remote limit switch source. User can choose between a remote
	 *            Talon SRX, CANifier, or deactivate the feature.
	 * @param normalOpenOrClose
	 *            Setting for normally open, normally closed, or disabled. This
	 *            setting matches the Phoenix Tuner drop down.
	 * @param deviceID
	 *            Device ID of remote source (Talon SRX or CANifier device ID).
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigForwardLimitSwitchSource(
			RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
			int deviceID, int timeoutMs = 0);
	/**
	 * Configures the reverse limit switch for a remote source. For example, a
	 * CAN motor controller may need to monitor the Limit-R pin of another Talon
	 * or CANifier.
	 *
	 * @param type
	 *            Remote limit switch source. User can choose between a remote
	 *            Talon SRX, CANifier, or deactivate the feature.
	 * @param normalOpenOrClose
	 *            Setting for normally open, normally closed, or disabled. This
	 *            setting matches the Phoenix Tuner drop down.
	 * @param deviceID
	 *            Device ID of remote source (Talon SRX or CANifier device ID).
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigReverseLimitSwitchSource(
			RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
			int deviceID, int timeoutMs = 0);
	/**
	 * Sets the enable state for limit switches.
	 *
	 * @param enable
	 *            Enable state for limit switches.
	 **/
	void OverrideLimitSwitchesEnable(bool enable);
	//------ local limit switch ----------//
	/**
	 * Configures a limit switch for a local/remote source.
	 *
	 * For example, a CAN motor controller may need to monitor the Limit-R pin
	 * of another Talon, CANifier, or local Gadgeteer feedback connector.
	 *
	 * If the sensor is remote, a device ID of zero is assumed. If that's not
	 * desired, use the four parameter version of this function.
	 *
	 * @param type
	 *            Limit switch source. User can choose
	 *            between the feedback connector, remote Talon SRX, CANifier, or
	 *            deactivate the feature.
	 * @param normalOpenOrClose
	 *            Setting for normally open, normally closed, or disabled. This
	 *            setting matches the Phoenix Tuner drop down.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigForwardLimitSwitchSource(LimitSwitchSource type,
			LimitSwitchNormal normalOpenOrClose, int timeoutMs = 0);
	/**
	 * Configures a limit switch for a local/remote source.
	 *
	 * For example, a CAN motor controller may need to monitor the Limit-R pin
	 * of another Talon, CANifier, or local Gadgeteer feedback connector.
	 *
	 * If the sensor is remote, a device ID of zero is assumed. If that's not
	 * desired, use the four parameter version of this function.
	 *
	 * @param type
	 *            Limit switch source. User can choose
	 *            between the feedback connector, remote Talon SRX, CANifier, or
	 *            deactivate the feature.
	 * @param normalOpenOrClose
	 *            Setting for normally open, normally closed, or disabled. This
	 *            setting matches the Phoenix Tuner drop down.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigReverseLimitSwitchSource(LimitSwitchSource type,
			LimitSwitchNormal normalOpenOrClose, int timeoutMs = 0);
	//------ soft limit ----------//
	/**
	 * Configures the forward soft limit threhold.
	 *
	 * @param forwardSensorLimit
	 *            Forward Sensor Position Limit (in raw sensor units).
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigForwardSoftLimitThreshold(int forwardSensorLimit,
			int timeoutMs = 0);
	/**
	 * Configures the reverse soft limit threshold.
	 *
	 * @param reverseSensorLimit
	 *            Reverse Sensor Position Limit (in raw sensor units).
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigReverseSoftLimitThreshold(int reverseSensorLimit,
			int timeoutMs = 0);
	/**
	 * Configures the forward soft limit enable.
	 *
	 * @param enable
	 *            Forward Sensor Position Limit Enable.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigForwardSoftLimitEnable(bool enable,
			int timeoutMs = 0);
	/**
	 * Configures the reverse soft limit enable.
	 *
	 * @param enable
	 *            Reverse Sensor Position Limit Enable.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigReverseSoftLimitEnable(bool enable,
			int timeoutMs = 0);		
	/**
	 * Can be used to override-disable the soft limits.
	 * This function can be used to quickly disable soft limits without
	 * having to modify the persistent configuration.
	 *
	 * @param enable
	 *            Enable state for soft limit switches.
	 */
	virtual void OverrideSoftLimitsEnable(bool enable);
	//------ Current Lim ----------//
	/* not available in base */
	//------ General Close loop ----------//
	/**
	 * Sets the 'P' constant in the given parameter slot.
	 *
	 * @param slotIdx
	 *            Parameter slot for the constant.
	 * @param value
	 *            Value of the P constant.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode Config_kP(int slotIdx, double value, int timeoutMs = 0);
	/**
	 * Sets the 'I' constant in the given parameter slot.
	 *
	 * @param slotIdx
	 *            Parameter slot for the constant.
	 * @param value
	 *            Value of the I constant.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode Config_kI(int slotIdx, double value, int timeoutMs = 0);
	/**
	 * Sets the 'D' constant in the given parameter slot.
	 *
	 * @param slotIdx
	 *            Parameter slot for the constant.
	 * @param value
	 *            Value of the D constant.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode Config_kD(int slotIdx, double value, int timeoutMs = 0);
	/**
	 * Sets the 'F' constant in the given parameter slot.
	 *
	 * @param slotIdx
	 *            Parameter slot for the constant.
	 * @param value
	 *            Value of the F constant.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode Config_kF(int slotIdx, double value, int timeoutMs = 0);
	/**
	 * Sets the Integral Zone constant in the given parameter slot. If the
	 * (absolute) closed-loop error is outside of this zone, integral
	 * accumulator is automatically cleared. This ensures than integral wind up
	 * events will stop after the sensor gets far enough from its target.
	 *
	 * @param slotIdx
	 *            Parameter slot for the constant.
	 * @param izone
	 *            Value of the Integral Zone constant (closed loop error units X
	 *            1ms).
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode Config_IntegralZone(int slotIdx, int izone,
			int timeoutMs = 0);
	/**
	 * Sets the allowable closed-loop error in the given parameter slot.
	 *
	 * @param slotIdx
	 *            Parameter slot for the constant.
	 * @param allowableCloseLoopError
	 *            Value of the allowable closed-loop error.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigAllowableClosedloopError(int slotIdx,
			int allowableCloseLoopError, int timeoutMs = 0);
	/**
	 * Sets the maximum integral accumulator in the given parameter slot.
	 *
	 * @param slotIdx
	 *            Parameter slot for the constant.
	 * @param iaccum
	 *            Value of the maximum integral accumulator (closed loop error
	 *            units X 1ms).
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigMaxIntegralAccumulator(int slotIdx, double iaccum,
			int timeoutMs = 0);
	/**
	 * Sets the peak closed-loop output.  This peak output is slot-specific and
	 *   is applied to the output of the associated PID loop.
	 * This setting is seperate from the generic Peak Output setting.
	 *
	 * @param slotIdx
	 *            Parameter slot for the constant.
	 * @param percentOut
	 *            Peak Percent Output from 0 to 1.  This value is absolute and
	 *						the magnitude will apply in both forward and reverse directions.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigClosedLoopPeakOutput(int slotIdx, double percentOut, int timeoutMs = 0);
	/**
	 * Sets the loop time (in milliseconds) of the PID closed-loop calculations.
	 * Default value is 1 ms.
	 *
	 * @param slotIdx
	 *            Parameter slot for the constant.
	 * @param loopTimeMs
	 *            Loop timing of the closed-loop calculations.  Minimum value of
	 *						1 ms, maximum of 64 ms.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigClosedLoopPeriod(int slotIdx, int loopTimeMs, int timeoutMs = 0);

	/**
	 * Configures the Polarity of the Auxiliary PID (PID1).
	 *
	 * Standard Polarity:
	 *    Primary Output = PID0 + PID1
	 *    Auxiliary Output = PID0 - PID1
	 *
	 * Inverted Polarity:
	 *    Primary Output = PID0 - PID1
	 *    Auxiliary Output = PID0 + PID1
	 *
	 * @param invert
	 *            If true, use inverted PID1 output polarity.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code
	 */
	virtual ctre::phoenix::ErrorCode ConfigAuxPIDPolarity(bool invert, int timeoutMs = 0);

	//------ Close loop State ----------//
	/**
	 * Sets the integral accumulator. Typically this is used to clear/zero the
	 * integral accumulator, however some use cases may require seeding the
	 * accumulator for a faster response.
	 *
	 * @param iaccum
	 *            Value to set for the integral accumulator (closed loop error
	 *            units X 1ms).
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode SetIntegralAccumulator(double iaccum, int pidIdx = 0,int timeoutMs = 0);
	/**
	 * Gets the closed-loop error. The units depend on which control mode is in
	 * use. See Phoenix-Documentation information on units.
	 *
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @return Closed-loop error value.
	 */
	virtual int GetClosedLoopError(int pidIdx = 0);
	/**
	 * Gets the iaccum value.
	 *
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @return Integral accumulator value (Closed-loop error X 1ms).
	 */
	virtual double GetIntegralAccumulator(int pidIdx = 0);
	/**
	 * Gets the derivative of the closed-loop error.
	 *
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @return The error derivative value.
	 */
	virtual double GetErrorDerivative(int pidIdx = 0);

	/**
	 * Selects which profile slot to use for closed-loop control.
	 *
	 * @param slotIdx
	 *            Profile slot to select.
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 **/
	virtual ctre::phoenix::ErrorCode SelectProfileSlot(int slotIdx, int pidIdx);

	/**
	 * Gets the current target of a given closed loop.
	 *
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @return The closed loop target.
	 */
	virtual double GetClosedLoopTarget(int pidIdx = 0);
	/**
	 * Gets the active trajectory target position using
	 * MotionMagic/MotionProfile control modes.
	 *
	 * @return The Active Trajectory Position in sensor units.
	 */	virtual int GetActiveTrajectoryPosition(int pidIdx = 0);
	/**
	 * Gets the active trajectory target velocity using
	 * MotionMagic/MotionProfile control modes.
	 *
	 * @return The Active Trajectory Velocity in sensor units per 100ms.
	 */
	virtual int GetActiveTrajectoryVelocity(int pidIdx = 0);	/**
	 * Gets the active trajectory arbitrary feedforward using
	 * MotionMagic/MotionProfile control modes.
	 *
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @return The Active Trajectory ArbFeedFwd in units of percent output
	 * 			(where 0.01 is 1%).
	 */	virtual double GetActiveTrajectoryArbFeedFwd(int pidIdx = 0);	/**
	 * Gets the active trajectory target heading using
	 * MotionMagicArc/MotionProfileArc control modes.
	 *
	 * @return The Active Trajectory Heading in degreees.
	 */
[[deprecated("Replaced by GetActiveTrajectoryPosition(1)")]]
	virtual double GetActiveTrajectoryHeading();

	//------ Motion Profile Settings used in Motion Magic  ----------//
	/**
	 * Sets the Motion Magic Cruise Velocity. This is the peak target velocity
	 * that the motion magic curve generator can use.
	 *
	 * @param sensorUnitsPer100ms
	 *            Motion Magic Cruise Velocity (in raw sensor units per 100 ms).
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigMotionCruiseVelocity(int sensorUnitsPer100ms,
			int timeoutMs = 0);
	/**
	 * Sets the Motion Magic Acceleration. This is the target acceleration that
	 * the motion magic curve generator can use.
	 *
	 * @param sensorUnitsPer100msPerSec
	 *            Motion Magic Acceleration (in raw sensor units per 100 ms per
	 *            second).
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigMotionAcceleration(int sensorUnitsPer100msPerSec,
			int timeoutMs = 0);
	//------ Motion Profile Buffer ----------//
	/**
	 * Clear the buffered motion profile in both controller's RAM (bottom), and in the
	 * API (top).
	 */
	virtual ErrorCode ClearMotionProfileTrajectories();
	/**
	 * Retrieve just the buffer count for the api-level (top) buffer. This
	 * routine performs no CAN or data structure lookups, so its fast and ideal
	 * if caller needs to quickly poll the progress of trajectory points being
	 * emptied into controller's RAM. Otherwise just use GetMotionProfileStatus.
	 *
	 * @return number of trajectory points in the top buffer.
	 */
	virtual int GetMotionProfileTopLevelBufferCount();
	/**
	 * Push another trajectory point into the top level buffer (which is emptied
	 * into the motor controller's bottom buffer as room allows).
	 * @param trajPt to push into buffer.
	 * The members should be filled in with these values...
	 *
	 * 		targPos:  servo position in sensor units.
	 *		targVel:  velocity to feed-forward in sensor units
	 *                 per 100ms.
	 * 		profileSlotSelect0  Which slot to get PIDF gains. PID is used for position servo. F is used
	 *						   as the Kv constant for velocity feed-forward. Typically this is hardcoded
	 *						   to the a particular slot, but you are free gain schedule if need be.
	 *						   Choose from [0,3]
	 *		profileSlotSelect1 Which slot to get PIDF gains for auxiliary PId.
	 *						   This only has impact during MotionProfileArc Control mode.
	 *						   Choose from [0,1].
	 * 	   isLastPoint  set to nonzero to signal motor controller to keep processing this
	 *                     trajectory point, instead of jumping to the next one
	 *                     when timeDurMs expires.  Otherwise MP executer will
	 *                     eventually see an empty buffer after the last point
	 *                     expires, causing it to assert the IsUnderRun flag.
	 *                     However this may be desired if calling application
	 *                     never wants to terminate the MP.
	 *		zeroPos  set to nonzero to signal motor controller to "zero" the selected
	 *                 position sensor before executing this trajectory point.
	 *                 Typically the first point should have this set only thus
	 *                 allowing the remainder of the MP positions to be relative to
	 *                 zero.
	 *		timeDur Duration to apply this trajectory pt.
	 * 				This time unit is ADDED to the exising base time set by
	 * 				configMotionProfileTrajectoryPeriod().
	 * @return CTR_OKAY if trajectory point push ok. ErrorCode if buffer is
	 *         full due to kMotionProfileTopBufferCapacity.
	 */
	virtual ctre::phoenix::ErrorCode PushMotionProfileTrajectory(const ctre::phoenix::motion::TrajectoryPoint & trajPt);
	/**
	 * @brief Simple one-shot firing of a complete MP.
	 * Starting in 2019, MPs can be fired by building a Buffered Trajectory Point Stream, and calling this routine.
	 *
	 * Once called, the motor controller software will automatically ...
	 * [1] Clear the firmware buffer of trajectory points.
	 * [2] Clear the underrun flags
	 * [3] Reset an index within the Buffered Trajectory Point Stream (so that the same profile can be run again and again).
	 * [4] Start a background thread to manage MP streaming (if not already running).
	 * [5a] If current control mode already matches motionProfControlMode, set MPE Output to "Hold".
	 * [5b] If current control mode does not matches motionProfControlMode, apply motionProfControlMode and set MPE Output to "Disable".
	 * [6] Stream the trajectory points into the device's firmware buffer.
	 * [7] Once motor controller has at least minBufferedPts worth in the firmware buffer, MP will automatically start (MPE Output set to "Enable").
	 * [8] Wait until MP finishes, then transitions the Motion Profile Executor's output to "Hold".
	 * [9] IsMotionProfileFinished() will now return true.
	 *
	 * Calling application can use IsMotionProfileFinished() to determine when internal state machine reaches [7].
	 * Calling application can cancel MP by calling set().  Otherwise do not call set() until MP has completed.
	 *
	 * The legacy API from previous years requires the calling application to pass points via the ProcessMotionProfileBuffer and PushMotionProfileTrajectory.
	 * This is no longer required if using this StartMotionProfile/IsMotionProfileFinished API.
	 *
	 * @param stream	A buffer that will be used to stream the trajectory points.  Caller can fill this container with the entire trajectory point, regardless of size.
	 * @param minBufferedPts	Minimum number of firmware buffered points before starting MP.  
	 *							Do not exceed device's firmware buffer capacity or MP will never fire (120 for Motion Profile, or 60 for Motion Profile Arc).
	 *							Recommendation value for this would be five to ten samples depending on timeDur of the trajectory point.
	 * @param motionProfControlMode		Pass MotionProfile or MotionProfileArc.
	 * @return nonzero error code if operation fails.
     */
	virtual ctre::phoenix::ErrorCode StartMotionProfile(ctre::phoenix::motion::BufferedTrajectoryPointStream & stream, uint32_t minBufferedPts, ControlMode motionProfControlMode);
	/**
	 * @brief Determine if running MP is complete.  
	 * This requires using the StartMotionProfile routine to start the MP.
	 * That is because managing the trajectory points is now done in a background thread (if StartMotionProfile is called).
	 *
	 * If calling application uses the legacy API  (more-complex buffering API) from previous years, than this API will
	 * not return true.
	 * 
	 * @return true if MP was started using StartMotionProfile, and it has completed execution (MPE is now in "hold").
	 */
	virtual bool IsMotionProfileFinished();
	/**
	 * Retrieve just the buffer full for the api-level (top) buffer. This
	 * routine performs no CAN or data structure lookups, so its fast and ideal
	 * if caller needs to quickly poll. Otherwise just use
	 * GetMotionProfileStatus.
	 *
	 * @return number of trajectory points in the top buffer.
	 */
	virtual bool IsMotionProfileTopLevelBufferFull();
	/**
	 * This must be called periodically to funnel the trajectory points from the
	 * API's top level buffer to the controller's bottom level buffer. Recommendation
	 * is to call this twice as fast as the execution rate of the motion
	 * profile. So if MP is running with 20ms trajectory points, try calling
	 * this routine every 10ms. All motion profile functions are thread-safe
	 * through the use of a mutex, so there is no harm in having the caller
	 * utilize threading.
	 */
	virtual void ProcessMotionProfileBuffer();
	/**
	 * Retrieve all status information.
	 * For best performance, Caller can snapshot all status information regarding the
	 * motion profile executer.
	 *
	 * @param statusToFill  Caller supplied object to fill.
	 *
	 * The members are filled, as follows...
	 *
	 *	topBufferRem:	The available empty slots in the trajectory buffer.
	 * 	 				The robot API holds a "top buffer" of trajectory points, so your applicaion
	 * 	 				can dump several points at once.  The API will then stream them into the
	 * 	 		 		low-level buffer, allowing the motor controller to act on them.
	 *
	 *	topBufferRem: The number of points in the top trajectory buffer.
	 *
	 *	btmBufferCnt: The number of points in the low level controller buffer.
	 *
	 *	hasUnderrun: 	Set if isUnderrun ever gets set.
	 * 	 	 	 	 	Only is cleared by clearMotionProfileHasUnderrun() to ensure
	 *
	 *	isUnderrun:		This is set if controller needs to shift a point from its buffer into
	 *					the active trajectory point however
	 *					the buffer is empty.
	 *					This gets cleared automatically when is resolved.
	 *
	 *	activePointValid:	True if the active trajectory point has not empty, false otherwise. The members in activePoint are only valid if this signal is set.
	 *
	 *	isLast:	is set/cleared based on the MP executer's current
	 *                trajectory point's IsLast value.  This assumes
	 *                IsLast was set when PushMotionProfileTrajectory
	 *                was used to insert the currently processed trajectory
	 *                point.
	 *
	 *	profileSlotSelect: The currently processed trajectory point's
	 *      			  selected slot.  This can differ in the currently selected slot used
	 *       				 for Position and Velocity servo modes
	 *
	 *	outputEnable:		The current output mode of the motion profile
	 *						executer (disabled, enabled, or hold).  When changing the set()
	 *						value in MP mode, it's important to check this signal to
	 *						confirm the change takes effect before interacting with the top buffer.
	 */
	virtual ctre::phoenix::ErrorCode GetMotionProfileStatus(ctre::phoenix::motion::MotionProfileStatus & statusToFill);
	/**
	 * Clear the "Has Underrun" flag. Typically this is called after application
	 * has confirmed an underrun had occured.
	 *
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ClearMotionProfileHasUnderrun(int timeoutMs = 0);
	/**
	 * Calling application can opt to speed up the handshaking between the robot
	 * API and the controller to increase the download rate of the controller's Motion
	 * Profile. Ideally the period should be no more than half the period of a
	 * trajectory point.
	 *
	 * @param periodMs
	 *            The transmit period in ms.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ChangeMotionControlFramePeriod(int periodMs);
	/**
	 * When trajectory points are processed in the motion profile executer, the MPE determines
	 * how long to apply the active trajectory point by summing baseTrajDurationMs with the
	 * timeDur of the trajectory point (see TrajectoryPoint).
	 *
	 * This allows general selection of the execution rate of the points with 1ms resolution,
	 * while allowing some degree of change from point to point.
	 * @param baseTrajDurationMs The base duration time of every trajectory point.
	 * 							This is summed with the trajectory points unique timeDur.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigMotionProfileTrajectoryPeriod(int baseTrajDurationMs, int timeoutMs = 0);
	/**
	 * When trajectory points are processed in the buffer, the motor controller can 
	 * linearly interpolate additional trajectory points between the buffered 
	 * points.  The time delta between these interpolated points is 1 ms.
	 * 
	 * By default this feature is enabled.
	 * 
	 * @param enable Whether to enable the trajectory point interpolation feature.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigMotionProfileTrajectoryInterpolationEnable(bool enable, int timeoutMs = 0);
	

    //------Feedback Device Interaction Settings---------//
    /**
     * Disables wrapping the position. If the signal goes from 1023 to 0 a motor 
     * controller will by default go to 1024. If wrapping the position is disabled,
     * it will go to 0;
     *
     * @param feedbackNotContinuous     disable wrapping the position. 
     *
     * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
    virtual ErrorCode ConfigFeedbackNotContinuous(bool feedbackNotContinuous, int timeoutMs = 0);
    /**
     * Disables going to neutral (brake/coast) when a remote sensor is no longer detected.
     *
     * @param remoteSensorClosedLoopDisableNeutralOnLOS     disable going to neutral 
     *
     * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
    virtual ErrorCode ConfigRemoteSensorClosedLoopDisableNeutralOnLOS(bool remoteSensorClosedLoopDisableNeutralOnLOS, int timeoutMs = 0);
    /**
     * Enables clearing the position of the feedback sensor when the forward 
     * limit switch is triggered
     *
     * @param clearPositionOnLimitF     Whether clearing is enabled, defaults false
     * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
    virtual ErrorCode ConfigClearPositionOnLimitF(bool clearPositionOnLimitF, int timeoutMs = 0);
    /**
     * Enables clearing the position of the feedback sensor when the reverse 
     * limit switch is triggered
     *
     * @param clearPositionOnLimitR     Whether clearing is enabled, defaults false
     * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
    virtual ErrorCode ConfigClearPositionOnLimitR(bool clearPositionOnLimitR, int timeoutMs = 0);
    /**
     * Enables clearing the position of the feedback sensor when the quadrature index signal
     * is detected
     *
     * @param clearPositionOnQuadIdx    Whether clearing is enabled, defaults false
     * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
    virtual ErrorCode ConfigClearPositionOnQuadIdx(bool clearPositionOnQuadIdx, int timeoutMs = 0);
    /**
     * Disables limit switches triggering (if enabled) when the sensor is no longer detected.
     *
     * @param limitSwitchDisableNeutralOnLOS    disable triggering
     *
     * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
    virtual ErrorCode ConfigLimitSwitchDisableNeutralOnLOS(bool limitSwitchDisableNeutralOnLOS, int timeoutMs = 0);
    /**
     * Disables soft limits triggering (if enabled) when the sensor is no longer detected.
     *
     * @param softLimitDisableNeutralOnLOS    disable triggering
     *
     * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
    virtual ErrorCode ConfigSoftLimitDisableNeutralOnLOS(bool softLimitDisableNeutralOnLOS, int timeoutMs = 0);
    /**
     * Sets the edges per rotation of a pulse width sensor. (This should be set for 
     * tachometer use).
     *
     * @param pulseWidthPeriod_EdgesPerRot    edges per rotation
     *
     * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
    virtual ErrorCode ConfigPulseWidthPeriod_EdgesPerRot(int pulseWidthPeriod_EdgesPerRot, int timeoutMs = 0);
    /**
     * Sets the number of samples to use in smoothing a pulse width sensor with a rolling 
     * average. Default is 1 (no smoothing).
     *
     * @param pulseWidthPeriod_FilterWindowSz   samples for rolling avg
     *
     * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
    virtual ErrorCode ConfigPulseWidthPeriod_FilterWindowSz(int pulseWidthPeriod_FilterWindowSz, int timeoutMs = 0);

    //------ error ----------//
	/**
	 * Gets the last error generated by this object. Not all functions return an
	 * error code but can potentially report errors. This function can be used
	 * to retrieve those error codes.
	 *
	 * @return Last Error Code generated by a function.
	 */
	virtual ctre::phoenix::ErrorCode GetLastError();
	//------ Faults ----------//
	/**
	 * Polls the various fault flags.
	 *
	 * @param toFill
	 *            Caller's object to fill with latest fault flags.
	 * @return Last Error Code generated by a function.
	 */
	virtual ctre::phoenix::ErrorCode GetFaults(Faults & toFill);
	/**
	 * Polls the various sticky fault flags.
	 *
	 * @param toFill
	 *            Caller's object to fill with latest sticky fault flags.
	 * @return Last Error Code generated by a function.
	 */
	virtual ctre::phoenix::ErrorCode GetStickyFaults(StickyFaults & toFill);
	/**
	 * Clears all sticky faults.
	 *
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Last Error Code generated by a function.
	 */
	virtual ctre::phoenix::ErrorCode ClearStickyFaults(int timeoutMs = 0);
	//------ Firmware ----------//
	/**
	 * Gets the firmware version of the device.
	 *
	 * @return Firmware version of device. For example: version 1-dot-2 is
	 *         0x0102.
	 */
	virtual int GetFirmwareVersion();
	/**
	 * Returns true if the device has reset since last call.
	 *
	 * @return Has a Device Reset Occurred?
	 */
	virtual bool HasResetOccurred();
	//------ Custom Persistent Params ----------//
	/**
	 * Sets the value of a custom parameter. This is for arbitrary use.
	 *
	 * Sometimes it is necessary to save calibration/limit/target information in
	 * the device. Particularly if the device is part of a subsystem that can be
	 * replaced.
	 *
	 * @param newValue
	 *            Value for custom parameter.
	 * @param paramIndex
	 *            Index of custom parameter [0,1]
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigSetCustomParam(int newValue, int paramIndex,
			int timeoutMs = 0);
	/**
	 * Gets the value of a custom parameter.
	 *
	 * @param paramIndex
	 *            Index of custom parameter [0,1].
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Value of the custom param.
	 */
	virtual int ConfigGetCustomParam(int paramIndex,
			int timeoutMs = 0);
	//------ Generic Param API, typically not used ----------//
	/**
	 * Sets a parameter. Generally this is not used. This can be utilized in -
	 * Using new features without updating API installation. - Errata
	 * workarounds to circumvent API implementation. - Allows for rapid testing
	 * / unit testing of firmware.
	 *
	 * @param param
	 *            Parameter enumeration.
	 * @param value
	 *            Value of parameter.
	 * @param subValue
	 *            Subvalue for parameter. Maximum value of 255.
	 * @param ordinal
	 *            Ordinal of parameter.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigSetParameter(ctre::phoenix::ParamEnum param, double value,
			uint8_t subValue, int ordinal, int timeoutMs = 0);
	/**
	 * Gets a parameter.
	 *
	 * @param param
	 *            Parameter enumeration.
	 * @param ordinal
	 *            Ordinal of parameter.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Value of parameter.
	 */
	virtual double ConfigGetParameter(ctre::phoenix::ParamEnum param, int ordinal, int timeoutMs = 0);
    virtual ErrorCode ConfigGetParameter(ParamEnum param, int32_t valueToSend,
            int32_t & valueReceived, uint8_t & subValue, int32_t ordinal,
            int32_t timeoutMs);
	//------ Misc. ----------//
	virtual int GetBaseID();
	/**
	 * @return control mode motor controller is in
	 */
	virtual ControlMode GetControlMode();
	// ----- Follower ------//
	/**
	 * Set the control mode and output value so that this motor controller will
	 * follow another motor controller. Currently supports following Victor SPX
	 * and Talon SRX.
	 *
	 * @param masterToFollow
	 *						Motor Controller object to follow.
	 * @param followerType
	 *						Type of following control.  Use AuxOutput1 to follow the master
	 *						device's auxiliary output 1.
	 *						Use PercentOutput for standard follower mode.
	 */
	void Follow(IMotorController & masterToFollow, ctre::phoenix::motorcontrol::FollowerType followerType);
	/**
	 * Set the control mode and output value so that this motor controller will
	 * follow another motor controller. Currently supports following Victor SPX
	 * and Talon SRX.
	 */
	virtual void Follow(IMotorController & masterToFollow);
	/**
	 * When master makes a device, this routine is called to signal the update.
	 */
	virtual void ValueUpdated();

	
	//-------Config All----------//
    /**
     * Gets all slot persistant settings.
     *
	 * @param slot        Object with all of the slot persistant settings
	 * @param slotIdx     Parameter slot for the constant.
     * @param timeoutMs
     *              Timeout value in ms. If nonzero, function will wait for
     *              config success and report an error if it times out.
     *              If zero, no blocking or checking is performed.
     */
	void GetSlotConfigs(SlotConfiguration &slot, int slotIdx = 0, int timeoutMs = 50);	
    /**
     * Gets all filter persistant settings.
     *
	 * @param filter        Object with all of the filter persistant settings
     * @param ordinal       0 for remote sensor 0 and 1 for remote sensor 1.
     * @param timeoutMs
     *              Timeout value in ms. If nonzero, function will wait for
     *              config success and report an error if it times out.
     *              If zero, no blocking or checking is performed.
     */
	void GetFilterConfigs(FilterConfiguration &Filter, int ordinal = 0, int timeoutMs = 50);
	
};// class BaseMotorController
} // namespace can
} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre
