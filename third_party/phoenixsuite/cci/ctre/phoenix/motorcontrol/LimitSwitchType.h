#pragma once

#include <string>

namespace ctre {
namespace phoenix {
namespace motorcontrol {

/** Limit switch source enum */
enum LimitSwitchSource {
	/**
	 * Limit switch directly connected to motor controller
	 */
	LimitSwitchSource_FeedbackConnector = 0,
	/**
	 * Use Limit switch connected to TalonSRX on CAN
	 */
	LimitSwitchSource_RemoteTalonSRX = 1,
	/**
	 * User Limit switch connected to CANifier
	 */
	LimitSwitchSource_RemoteCANifier = 2,
	/**
	 * Don't use a limit switch
	 */
	LimitSwitchSource_Deactivated = 3,
};

/** Remote Limit switch source enum */
enum RemoteLimitSwitchSource {
	/**
	 * Don't use limit switch, this is the factory default value
	 */
	RemoteLimitSwitchSource_FactoryDefaultOff = 0,
	/**
	 * Use Limit switch connected to TalonSRX on CAN
	 */
	RemoteLimitSwitchSource_RemoteTalonSRX = 1,
	/**
	 * User Limit switch connected to CANifier
	 */
	RemoteLimitSwitchSource_RemoteCANifier = 2,
	/**
	 * Don't use a limit switch
	 */
	RemoteLimitSwitchSource_Deactivated = 3,
};

/**
 * Choose whether the limit switch is normally
 * open or normally closed
 */
enum LimitSwitchNormal {
    /**
     * Limit Switch is tripped when
     * the circuit is closed
     */
	LimitSwitchNormal_NormallyOpen = 0,
    /**
     * Limit Switch is tripped when
     * the circuit is open 
     */
	LimitSwitchNormal_NormallyClosed = 1,
    /**
     * Limit switch is disabled 
     */ 
	LimitSwitchNormal_Disabled = 2
};

/**
 * Class to handle various functions regarding limit switches
 */
class LimitSwitchRoutines {
public:
	/**
	 * Takes a RemoteLimitSwitchSource and brings it up to a LimitSwitchSource
	 * @param limitSwitchSource LimitSwitchSource to promote
	 * @return promoted limitSwitchSource
	 */
	static LimitSwitchSource Promote(
			RemoteLimitSwitchSource limitSwitchSource) {
		return (LimitSwitchSource) limitSwitchSource;
	}
	//Checks if a limit switch is a one of the remote values 
	//(i.e. RemoteTalonSRX or RemoteCANifier)
	/**
	 * Checks if a limit switch is one of the remote values
	 * (i.e. RemoteTalonSRX or RemoteCANifier)
	 *
	 * @param limitSwitchSource limitSwitchSource to check
	 * @return true if it's a remote limit switch source
	 */
	static bool IsRemote(LimitSwitchSource limitSwitchSource)
	{
		return limitSwitchSource > 0 && limitSwitchSource < 3;
	}
	/**
	 * @param value LimitSwitchSource to get the string value of
	 * @return string representation of value
	 */
    static std::string toString(LimitSwitchSource value) {
        switch(value) {
            case LimitSwitchSource_FeedbackConnector : return "LimitSwitchSource_FeedbackConnector";
            case LimitSwitchSource_RemoteTalonSRX : return "LimitSwitchSource_RemoteTalonSRX";
            case LimitSwitchSource_RemoteCANifier : return "LimitSwitchSource_RemoteCANifier";
            case LimitSwitchSource_Deactivated : return "LimitSwitchSource_Deactivated";
            default : return "InvalidValue";
        }

    }
	/**
	 * @param value LimitSwitchSource to get the string value of
	 * @return string representation of value
	 */
    static std::string toString(RemoteLimitSwitchSource value) {
        switch(value) {
            case RemoteLimitSwitchSource_FactoryDefaultOff: return "None (factory default value)";
            case RemoteLimitSwitchSource_RemoteTalonSRX : return "RemoteLimitSwitchSource_RemoteTalonSRX";
            case RemoteLimitSwitchSource_RemoteCANifier : return "RemoteLimitSwitchSource_RemoteCANifier";
            case RemoteLimitSwitchSource_Deactivated : return "RemoteLimitSwitchSource_Deactivated";
            default : return "InvalidValue";
        }

    }
	/**
	 * @param value LimitSwitchNormal to get the string value of
	 * @return string representation of value
	 */
    static std::string toString(LimitSwitchNormal value) {
        switch(value) {
            case LimitSwitchNormal_NormallyOpen : return "LimitSwitchNormal_NormallyOpen";
            case LimitSwitchNormal_NormallyClosed : return "LimitSwitchNormal_NormallyClosed";
            case LimitSwitchNormal_Disabled : return "LimitSwitchNormal_Disabled";
            default : return "InvalidValue";
        }

    }

};
} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre
