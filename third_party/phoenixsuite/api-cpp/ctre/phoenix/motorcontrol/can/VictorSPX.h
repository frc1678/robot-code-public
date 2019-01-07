#pragma once
#if defined(WIN32) || defined(_WIN32) || defined(_WIN64)
#pragma warning (push)
#pragma warning (disable : 4250)
#endif

#include "ctre/phoenix/motorcontrol/can/BaseMotorController.h"
#include "ctre/phoenix/CustomParamConfiguration.h"
//#include "ctre/phoenix/motorcontrol/IMotorController.h"

namespace ctre {
namespace phoenix {
namespace motorcontrol{
namespace can {

/**
 * VEX Victor SPX Motor Configuration settings.
 */
struct VictorSPXPIDSetConfiguration : BasePIDSetConfiguration {
	RemoteFeedbackDevice selectedFeedbackSensor;

	VictorSPXPIDSetConfiguration() :
		selectedFeedbackSensor(RemoteFeedbackDevice_RemoteSensor0) 
        //NOTE: while the factory default value is 0, this value can't
        //be set by the API. Thus, RemoteSensor0 is the default
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

struct VictorSPXPIDSetConfigUtil {
	private:
		static VictorSPXPIDSetConfiguration _default;
	public:
		static bool SelectedFeedbackSensorDifferent (const VictorSPXPIDSetConfiguration & settings) { return (!(settings.selectedFeedbackSensor == _default.selectedFeedbackSensor)); }
		static bool SelectedFeedbackCoefficientDifferent (const VictorSPXPIDSetConfiguration & settings) { return (!(settings.selectedFeedbackCoefficient == _default.selectedFeedbackCoefficient)); }
};

struct VictorSPXConfiguration : BaseMotorControllerConfiguration {
	VictorSPXPIDSetConfiguration primaryPID;	
	VictorSPXPIDSetConfiguration auxiliaryPID;	
	RemoteLimitSwitchSource forwardLimitSwitchSource;
	RemoteLimitSwitchSource reverseLimitSwitchSource;
	int forwardLimitSwitchDeviceID; //Limit Switch device id isn't used unless device is a remote
	int reverseLimitSwitchDeviceID;
	LimitSwitchNormal forwardLimitSwitchNormal;
	LimitSwitchNormal reverseLimitSwitchNormal;
	RemoteFeedbackDevice sum0Term;
	RemoteFeedbackDevice sum1Term;
	RemoteFeedbackDevice diff0Term;
	RemoteFeedbackDevice diff1Term;

	VictorSPXConfiguration() :
		forwardLimitSwitchSource(RemoteLimitSwitchSource_Deactivated), 
		reverseLimitSwitchSource(RemoteLimitSwitchSource_Deactivated),
        forwardLimitSwitchDeviceID(0),
        reverseLimitSwitchDeviceID(0),
        forwardLimitSwitchNormal(LimitSwitchNormal_NormallyOpen), 
        reverseLimitSwitchNormal(LimitSwitchNormal_NormallyOpen), 
		sum0Term (RemoteFeedbackDevice_RemoteSensor0), 
		sum1Term (RemoteFeedbackDevice_RemoteSensor0), 
		diff0Term(RemoteFeedbackDevice_RemoteSensor0),
		diff1Term(RemoteFeedbackDevice_RemoteSensor0)
        //NOTE: while the factory default value is 0, this value can't
        //be set by the API. Thus, RemoteSensor0 is the default

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
        retstr += BaseMotorControllerConfiguration::toString(prependString);
        
        return retstr;
    }
};

class VictorConfigUtil {
	private:
		static struct VictorSPXConfiguration _default;
	public:
		static bool ForwardLimitSwitchSourceDifferent (const VictorSPXConfiguration & settings) { return (!(settings.forwardLimitSwitchSource == _default.forwardLimitSwitchSource)) || !settings.enableOptimizations; }
		static bool ReverseLimitSwitchSourceDifferent (const VictorSPXConfiguration & settings) { return (!(settings.reverseLimitSwitchSource == _default.reverseLimitSwitchSource)) || !settings.enableOptimizations; }
		static bool ForwardLimitSwitchDeviceIDDifferent (const VictorSPXConfiguration & settings) { return (!(settings.forwardLimitSwitchDeviceID == _default.forwardLimitSwitchDeviceID)) || !settings.enableOptimizations; }
		static bool ReverseLimitSwitchDeviceIDDifferent (const VictorSPXConfiguration & settings) { return (!(settings.reverseLimitSwitchDeviceID == _default.reverseLimitSwitchDeviceID)) || !settings.enableOptimizations; }
		static bool ForwardLimitSwitchNormalDifferent (const VictorSPXConfiguration & settings) { return (!(settings.forwardLimitSwitchNormal == _default.forwardLimitSwitchNormal)) || !settings.enableOptimizations; }
		static bool ReverseLimitSwitchNormalDifferent (const VictorSPXConfiguration & settings) { return (!(settings.reverseLimitSwitchNormal == _default.reverseLimitSwitchNormal)) || !settings.enableOptimizations; }
		static bool Sum0TermDifferent (const VictorSPXConfiguration & settings) { return (!(settings.sum0Term == _default.sum0Term)) || !settings.enableOptimizations; }
		static bool Sum1TermDifferent (const VictorSPXConfiguration & settings) { return (!(settings.sum1Term == _default.sum1Term)) || !settings.enableOptimizations; }
		static bool Diff0TermDifferent (const VictorSPXConfiguration & settings) { return (!(settings.diff0Term == _default.diff0Term)) || !settings.enableOptimizations; }
		static bool Diff1TermDifferent (const VictorSPXConfiguration & settings) { return (!(settings.diff1Term == _default.diff1Term)) || !settings.enableOptimizations; }
		
		static bool ForwardLimitSwitchDifferent (const VictorSPXConfiguration & settings) {
			return ForwardLimitSwitchDeviceIDDifferent(settings) || ForwardLimitSwitchNormalDifferent(settings) || ForwardLimitSwitchSourceDifferent(settings);
		}
		static bool ReverseLimitSwitchDifferent (const VictorSPXConfiguration & settings) {
			return ReverseLimitSwitchDeviceIDDifferent(settings) || ReverseLimitSwitchNormalDifferent(settings) || ReverseLimitSwitchSourceDifferent(settings);
		}
};

/**
 * VEX Victor SPX Motor Controller when used on CAN Bus.
 */
class VictorSPX: public virtual ctre::phoenix::motorcontrol::can::BaseMotorController,
                 public virtual ctre::phoenix::motorcontrol::IMotorController {

private:
	ctre::phoenix::ErrorCode ConfigurePID(const VictorSPXPIDSetConfiguration &pid, int pidIdx, int timeoutMs, bool enableOptimizations);
public:
	/**
	 * Constructor
	 * 
	 * @param deviceNumber
	 *            [0,62]
	 */
	VictorSPX(int deviceNumber);
	virtual ~VictorSPX() {
	}
	VictorSPX(VictorSPX const&) = delete;
	VictorSPX& operator=(VictorSPX const&) = delete;
	
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
	void GetPIDConfigs(VictorSPXPIDSetConfiguration &pid, int pidIdx = 0, int timeoutMs = 50);
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
	virtual ctre::phoenix::ErrorCode ConfigAllSettings(const VictorSPXConfiguration &allConfigs, int timeoutMs = 50);
    /**
     * Gets all persistant settings.
     *
	 * @param allConfigs        Object with all of the persistant settings
     * @param timeoutMs
     *              Timeout value in ms. If nonzero, function will wait for
     *              config success and report an error if it times out.
     *              If zero, no blocking or checking is performed.
     */
	virtual void GetAllConfigs(VictorSPXConfiguration &allConfigs, int timeoutMs = 50);

};// class VictorSPX
} // namespace can
} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre

#if defined(WIN32) || defined(_WIN32) || defined(_WIN64)
#pragma warning (pop)
#endif
