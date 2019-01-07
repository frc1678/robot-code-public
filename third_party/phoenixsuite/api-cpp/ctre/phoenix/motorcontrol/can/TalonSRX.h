#pragma once
#if defined(WIN32) || defined(_WIN32) || defined(_WIN64)
#pragma warning (push)
#pragma warning (disable : 4250)
#endif

#include "ctre/phoenix/motorcontrol/can/BaseMotorController.h"
#include "ctre/phoenix/motorcontrol/IMotorControllerEnhanced.h"
#include "ctre/phoenix/CustomParamConfiguration.h"

/* forward proto's */
namespace ctre {
namespace phoenix {
namespace motorcontrol {
class SensorCollection;
}
}
}

namespace ctre {
namespace phoenix {
namespace motorcontrol {
namespace can {

/**
 * CTRE Talon SRX Motor Configuration settings.
 */

struct TalonSRXPIDSetConfiguration : BasePIDSetConfiguration {
    FeedbackDevice selectedFeedbackSensor;

    TalonSRXPIDSetConfiguration() :
        selectedFeedbackSensor(QuadEncoder)
    {
    }

	std::string toString() {
		return toString("");
	}

    std::string toString(std::string prependString) {

        std::string retstr = prependString + ".selectedFeedbackSensor = " + FeedbackDeviceRoutines::toString(selectedFeedbackSensor) + ";\n";
        retstr += BasePIDSetConfiguration::toString(prependString);
        return retstr;
    }
};

struct TalonSRXPIDSetConfigUtil {
	private:
		static TalonSRXPIDSetConfiguration _default;
	public:
		static bool SelectedFeedbackSensorDifferent (const TalonSRXPIDSetConfiguration & settings) { return (!(settings.selectedFeedbackSensor == _default.selectedFeedbackSensor)); }
		static bool SelectedFeedbackCoefficientDifferent (const TalonSRXPIDSetConfiguration & settings) { return (!(settings.selectedFeedbackCoefficient == _default.selectedFeedbackCoefficient)); }
};


struct TalonSRXConfiguration : BaseMotorControllerConfiguration{
	TalonSRXPIDSetConfiguration primaryPID;
	TalonSRXPIDSetConfiguration auxiliaryPID;
	LimitSwitchSource forwardLimitSwitchSource;
	LimitSwitchSource reverseLimitSwitchSource;
	int forwardLimitSwitchDeviceID; //Limit Switch device id isn't used unless device is a remote
	int reverseLimitSwitchDeviceID;
	LimitSwitchNormal forwardLimitSwitchNormal;
	LimitSwitchNormal reverseLimitSwitchNormal;
    FeedbackDevice sum0Term;
	FeedbackDevice sum1Term;
	FeedbackDevice diff0Term;
	FeedbackDevice diff1Term;
	int peakCurrentLimit; 
    int peakCurrentDuration;
    int continuousCurrentLimit; 
    TalonSRXConfiguration() :
		forwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector),
		reverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector),
        forwardLimitSwitchDeviceID(0),
        reverseLimitSwitchDeviceID(0),
        forwardLimitSwitchNormal(LimitSwitchNormal_NormallyOpen), 
        reverseLimitSwitchNormal(LimitSwitchNormal_NormallyOpen), 
		sum0Term (QuadEncoder),
		sum1Term (QuadEncoder),
		diff0Term(QuadEncoder),
		diff1Term(QuadEncoder),
        peakCurrentLimit(1),
		peakCurrentDuration(1), 
		continuousCurrentLimit(1)
	{
	}

	std::string toString() {
		return toString("");
	}

    std::string toString(std::string prependString) {


        std::string retstr = primaryPID.toString(prependString + ".primaryPID");
	    retstr += auxiliaryPID.toString(prependString + ".auxiliaryPID");
	    retstr += prependString + ".forwardLimitSwitchSource = " + LimitSwitchRoutines::toString(forwardLimitSwitchSource) + ";\n";
	    retstr += prependString + ".reverseLimitSwitchSource = " + LimitSwitchRoutines::toString(reverseLimitSwitchSource) + ";\n";
        retstr += prependString + ".forwardLimitSwitchDeviceID = " + std::to_string(forwardLimitSwitchDeviceID) + ";\n";
        retstr += prependString + ".reverseLimitSwitchDeviceID = " + std::to_string(reverseLimitSwitchDeviceID) + ";\n";
        retstr += prependString + ".forwardLimitSwitchNormal = " + LimitSwitchRoutines::toString(forwardLimitSwitchNormal) + ";\n";
        retstr += prependString + ".reverseLimitSwitchNormal = " + LimitSwitchRoutines::toString(reverseLimitSwitchNormal) + ";\n";
	    retstr += prependString + ".sum0Term = " + FeedbackDeviceRoutines::toString(sum0Term) + ";\n";
	    retstr += prependString + ".sum1Term = " + FeedbackDeviceRoutines::toString(sum1Term) + ";\n";
	    retstr += prependString + ".diff0Term = " + FeedbackDeviceRoutines::toString(diff0Term) + ";\n";
	    retstr += prependString + ".diff1Term = " + FeedbackDeviceRoutines::toString(diff1Term) + ";\n";
	    retstr += prependString + ".peakCurrentLimit = " + std::to_string(peakCurrentLimit) + ";\n"; 
        retstr += prependString + ".peakCurrentDuration = " + std::to_string(peakCurrentDuration) + ";\n";
        retstr += prependString + ".continuousCurrentLimit = " + std::to_string(continuousCurrentLimit) + ";\n"; 
         retstr += BaseMotorControllerConfiguration::toString(prependString);

       return retstr; 
    }
};// struct TalonSRXConfiguration

class TalonConfigUtil {
	private:
		static struct TalonSRXConfiguration _default;
	public:
		static bool ForwardLimitSwitchSourceDifferent (const TalonSRXConfiguration & settings) { return (!(settings.forwardLimitSwitchSource == _default.forwardLimitSwitchSource)) || !settings.enableOptimizations; }
		static bool ReverseLimitSwitchSourceDifferent (const TalonSRXConfiguration & settings) { return (!(settings.reverseLimitSwitchSource == _default.reverseLimitSwitchSource)) || !settings.enableOptimizations; }
		static bool ForwardLimitSwitchDeviceIDDifferent (const TalonSRXConfiguration & settings) { return (!(settings.forwardLimitSwitchDeviceID == _default.forwardLimitSwitchDeviceID)) || !settings.enableOptimizations; }
		static bool ReverseLimitSwitchDeviceIDDifferent (const TalonSRXConfiguration & settings) { return (!(settings.reverseLimitSwitchDeviceID == _default.reverseLimitSwitchDeviceID)) || !settings.enableOptimizations; }
		static bool ForwardLimitSwitchNormalDifferent (const TalonSRXConfiguration & settings) { return (!(settings.forwardLimitSwitchNormal == _default.forwardLimitSwitchNormal)) || !settings.enableOptimizations; }
		static bool ReverseLimitSwitchNormalDifferent (const TalonSRXConfiguration & settings) { return (!(settings.reverseLimitSwitchNormal == _default.reverseLimitSwitchNormal)) || !settings.enableOptimizations; }
		static bool Sum0TermDifferent (const TalonSRXConfiguration & settings) { return (!(settings.sum0Term == _default.sum0Term)) || !settings.enableOptimizations; }
		static bool Sum1TermDifferent (const TalonSRXConfiguration & settings) { return (!(settings.sum1Term == _default.sum1Term)) || !settings.enableOptimizations; }
		static bool Diff0TermDifferent (const TalonSRXConfiguration & settings) { return (!(settings.diff0Term == _default.diff0Term)) || !settings.enableOptimizations; }
		static bool Diff1TermDifferent (const TalonSRXConfiguration & settings) { return (!(settings.diff1Term == _default.diff1Term)) || !settings.enableOptimizations; }
		static bool PeakCurrentLimitDifferent (const TalonSRXConfiguration & settings) { return (!(settings.peakCurrentLimit == _default.peakCurrentLimit)) || !settings.enableOptimizations; }
		static bool PeakCurrentDurationDifferent (const TalonSRXConfiguration & settings) { return (!(settings.peakCurrentDuration == _default.peakCurrentDuration)) || !settings.enableOptimizations; }
		static bool ContinuousCurrentLimitDifferent (const TalonSRXConfiguration & settings) { return (!(settings.continuousCurrentLimit == _default.continuousCurrentLimit)) || !settings.enableOptimizations; }

		static bool ForwardLimitSwitchDifferent (const TalonSRXConfiguration & settings) {
			return ForwardLimitSwitchDeviceIDDifferent(settings) || ForwardLimitSwitchNormalDifferent(settings) || ForwardLimitSwitchSourceDifferent(settings);
		}
		static bool ReverseLimitSwitchDifferent (const TalonSRXConfiguration & settings) {
			return ReverseLimitSwitchDeviceIDDifferent(settings) || ReverseLimitSwitchNormalDifferent(settings) || ReverseLimitSwitchSourceDifferent(settings);
		}
};

/**
 * CTRE Talon SRX Motor Controller when used on CAN Bus.
 */

class TalonSRX: public virtual BaseMotorController,
		public virtual IMotorControllerEnhanced {
private:
	ctre::phoenix::motorcontrol::SensorCollection * _sensorColl;

	ctre::phoenix::ErrorCode ConfigurePID(const TalonSRXPIDSetConfiguration &pid, int pidIdx, int timeoutMs, bool enableOptimizations);
public:
	TalonSRX(int deviceNumber);
	~TalonSRX();
	TalonSRX() = delete;
	TalonSRX(TalonSRX const&) = delete;
	TalonSRX& operator=(TalonSRX const&) = delete;

	virtual ctre::phoenix::ErrorCode ConfigSelectedFeedbackSensor(FeedbackDevice feedbackDevice, int pidIdx = 0, int timeoutMs = 0);
	virtual ctre::phoenix::ErrorCode ConfigSelectedFeedbackSensor(RemoteFeedbackDevice feedbackDevice, int pidIdx = 0, int timeoutMs = 0);

	/**
	 * Sets the period of the given status frame.
	 *
	 * User ensure CAN Bus utilization is not high.
	 *
	 * This setting is not persistent and is lost when device is reset.
	 * If this is a concern, calling application can use HasReset()
	 * to determine if the status frame needs to be reconfigured.
	 *
	 * @param frame
	 *            Frame whose period is to be changed.
	 * @param periodMs
	 *            Period in ms for the given frame.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode SetStatusFramePeriod(StatusFrameEnhanced frame,uint8_t periodMs, int timeoutMs = 0);
	virtual ctre::phoenix::ErrorCode SetStatusFramePeriod(StatusFrame frame,uint8_t periodMs, int timeoutMs = 0);

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
	virtual int GetStatusFramePeriod(StatusFrame frame, int timeoutMs = 0);
	
	//------ General Status ----------//
    /**
	 * Gets the output current of the motor controller.
	 *
	 * @return The output current (in amps).
	 */
    virtual double GetOutputCurrent();

	//------ Velocity measurement ----------//
	/**
	 * Configures the period of each velocity sample.
	 * Every 1ms a position value is sampled, and the delta between that sample
	 * and the position sampled kPeriod ms ago is inserted into a filter.
	 * kPeriod is configured with this function.
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
	 *            measurement. Valid values are 1,2,4,8,16,32. If another
	 *            value is specified, it will truncate to nearest support value.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigVelocityMeasurementWindow(int windowSize,
			int timeoutMs = 0);

	//------ limit switch ----------//
	/**
	 * Configures a limit switch for a local/remote source.
	 *
	 * For example, a CAN motor controller may need to monitor the Limit-R pin
	 * of another Talon, CANifier, or local Gadgeteer feedback connector.
	 *
	 * If the sensor is remote, a device ID of zero is assumed.
	 * If that's not desired, use the four parameter version of this function.
	 *
	 * @param limitSwitchSource
	 *            Limit switch source.
	 *            User can choose between the feedback connector, remote Talon SRX, CANifier, or deactivate the feature.
	 * @param normalOpenOrClose
	 *            Setting for normally open, normally closed, or disabled. This setting
	 *            matches the Phoenix Tuner drop down.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigForwardLimitSwitchSource(
			LimitSwitchSource limitSwitchSource,
			LimitSwitchNormal normalOpenOrClose, int timeoutMs = 0);
	/**
	 * Configures a limit switch for a local/remote source.
	 *
	 * For example, a CAN motor controller may need to monitor the Limit-R pin
	 * of another Talon, CANifier, or local Gadgeteer feedback connector.
	 *
	 * If the sensor is remote, a device ID of zero is assumed.
	 * If that's not desired, use the four parameter version of this function.
	 *
	 * @param limitSwitchSource
	 *            Limit switch source.
	 *            User can choose between the feedback connector, remote Talon SRX, CANifier, or deactivate the feature.
	 * @param normalOpenOrClose
	 *            Setting for normally open, normally closed, or disabled. This setting
	 *            matches the Phoenix Tuner drop down.
	 * @param deviceID
	 *            Device ID of remote source (Talon SRX or CANifier device ID).
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ctre::phoenix::ErrorCode ConfigForwardLimitSwitchSource(
			RemoteLimitSwitchSource limitSwitchSource,
			LimitSwitchNormal normalOpenOrClose, int deviceID, int timeoutMs = 0);
	/**
	 * Configures a limit switch for a local/remote source.
	 *
	 * For example, a CAN motor controller may need to monitor the Limit-R pin
	 * of another Talon, CANifier, or local Gadgeteer feedback connector.
	 *
	 * If the sensor is remote, a device ID of zero is assumed. If that's not
	 * desired, use the four parameter version of this function.
	 *
	 * @param limitSwitchSource
	 *            Limit switch source. @see #LimitSwitchSource User can choose
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
	virtual ctre::phoenix::ErrorCode ConfigReverseLimitSwitchSource(
			LimitSwitchSource limitSwitchSource,
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
	 * @param limitSwitchSource
	 *            Limit switch source. @see #LimitSwitchSource User can choose
	 *            between the feedback connector, remote Talon SRX, CANifier, or
	 *            deactivate the feature.
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
			RemoteLimitSwitchSource limitSwitchSource,
			LimitSwitchNormal normalOpenOrClose, int deviceID, int timeoutMs = 0);

	//------ Current Limit ----------//
	/**
	 * Configure the peak allowable current (when current limit is enabled).
	 * 
	 * Current limit is activated when current exceeds the peak limit for longer
	 * than the peak duration. Then software will limit to the continuous limit.
	 * This ensures current limiting while allowing for momentary excess current
	 * events.
	 *
	 * For simpler current-limiting (single threshold) use
	 * ConfigContinuousCurrentLimit() and set the peak to zero:
	 * ConfigPeakCurrentLimit(0).
	 * 
	 * @param amps
	 *            Amperes to limit.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 */
	virtual ctre::phoenix::ErrorCode ConfigPeakCurrentLimit(int amps, int timeoutMs = 0);
	/**
	 * Configure the peak allowable duration (when current limit is enabled).
	 *
	 * Current limit is activated when current exceeds the peak limit for longer
	 * than the peak duration. Then software will limit to the continuous limit.
	 * This ensures current limiting while allowing for momentary excess current
	 * events.
	 *
	 * For simpler current-limiting (single threshold) use
	 * ConfigContinuousCurrentLimit() and set the peak to zero:
	 * ConfigPeakCurrentLimit(0).
	 * 
	 * @param milliseconds
	 *            How long to allow current-draw past peak limit.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 */
	virtual ctre::phoenix::ErrorCode ConfigPeakCurrentDuration(int milliseconds,
			int timeoutMs = 0);
	/**
	 * Configure the continuous allowable current-draw (when current limit is
	 * enabled).
	 *
	 * Current limit is activated when current exceeds the peak limit for longer
	 * than the peak duration. Then software will limit to the continuous limit.
	 * This ensures current limiting while allowing for momentary excess current
	 * events.
	 *
	 * For simpler current-limiting (single threshold) use
	 * ConfigContinuousCurrentLimit() and set the peak to zero:
	 * ConfigPeakCurrentLimit(0).
	 * 
	 * @param amps
	 *            Amperes to limit.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 */
	virtual ctre::phoenix::ErrorCode ConfigContinuousCurrentLimit(int amps, int timeoutMs = 0);
	/**
	 * Enable or disable Current Limit.
	 * 
	 * @param enable
	 *            Enable state of current limit.
	 * @see configPeakCurrentLimit()
	 * @see configPeakCurrentDuration()
	 * @see configContinuousCurrentLimit()
	 */
	virtual void EnableCurrentLimit(bool enable);
	
	//------ RAW Sensor API ----------//
	/**
	 * @return object that can get/set individual RAW sensor values.
	 */
	ctre::phoenix::motorcontrol::SensorCollection & GetSensorCollection();

	//------ All Configs ----------//
    /**
     * Gets all PID set persistant settings.
     *
	 * @param pid               Object with all of the PID set persistant settings
	 * @param pidIdx            0 for Primary closed-loop. 1 for auxiliary closed-loop.
     * @param timeoutMs
     *              Timeout value in ms. If nonzero, function will wait for
     *              config success and report an error if it times out.
     *              If zero, no blocking or checking is performed.
     */
	void GetPIDConfigs(TalonSRXPIDSetConfiguration &pid, int pidIdx = 0, int timeoutMs = 50);
    /**
     * Configures all peristant settings.
     *
	 * @param allConfigs        Object with all of the persistant settings
     * @param timeoutMs
     *              Timeout value in ms. If nonzero, function will wait for
     *              config success and report an error if it times out.
     *              If zero, no blocking or checking is performed.
     *
     * @return Error Code generated by function. 0 indicates no error. 
     */
	ctre::phoenix::ErrorCode ConfigAllSettings(const TalonSRXConfiguration &allConfigs, int timeoutMs = 50);
    /**
     * Gets all persistant settings.
     *
	 * @param allConfigs        Object with all of the persistant settings
     * @param timeoutMs
     *              Timeout value in ms. If nonzero, function will wait for
     *              config success and report an error if it times out.
     *              If zero, no blocking or checking is performed.
     */
	void GetAllConfigs(TalonSRXConfiguration &allConfigs, int timeoutMs = 50);
};// class TalonSRX




} // namespace can
} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre

#if defined(WIN32) || defined(_WIN32) || defined(_WIN64)
#pragma warning (pop)
#endif
