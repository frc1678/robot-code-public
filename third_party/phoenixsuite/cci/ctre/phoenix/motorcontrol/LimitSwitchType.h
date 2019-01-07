#pragma once

#include <string>

namespace ctre {
namespace phoenix {
namespace motorcontrol {

enum LimitSwitchSource {
	LimitSwitchSource_FeedbackConnector = 0, /* default */
	LimitSwitchSource_RemoteTalonSRX = 1,
	LimitSwitchSource_RemoteCANifier = 2,
	LimitSwitchSource_Deactivated = 3,
};

enum RemoteLimitSwitchSource {
	RemoteLimitSwitchSource_FactoryDefaultOff = 0,
	RemoteLimitSwitchSource_RemoteTalonSRX = 1,
	RemoteLimitSwitchSource_RemoteCANifier = 2,
	RemoteLimitSwitchSource_Deactivated = 3,
};

enum LimitSwitchNormal {
	LimitSwitchNormal_NormallyOpen = 0,
	LimitSwitchNormal_NormallyClosed = 1,
	LimitSwitchNormal_Disabled = 2
};

class LimitSwitchRoutines {
public:
	static LimitSwitchSource Promote(
			RemoteLimitSwitchSource limitSwitchSource) {
		return (LimitSwitchSource) limitSwitchSource;
	}
	//Checks if a limit switch is a one of the remote values 
	//(i.e. RemoteTalonSRX or RemoteCANifier)
	static bool IsRemote(LimitSwitchSource limitSwitchSource)
	{
		return limitSwitchSource > 0 && limitSwitchSource < 3;
	}
    static std::string toString(LimitSwitchSource value) {
        switch(value) {
            case LimitSwitchSource_FeedbackConnector : return "LimitSwitchSource_FeedbackConnector";
            case LimitSwitchSource_RemoteTalonSRX : return "LimitSwitchSource_RemoteTalonSRX";
            case LimitSwitchSource_RemoteCANifier : return "LimitSwitchSource_RemoteCANifier";
            case LimitSwitchSource_Deactivated : return "LimitSwitchSource_Deactivated";
            default : return "InvalidValue";
        }

    }
    static std::string toString(RemoteLimitSwitchSource value) {
        switch(value) {
            case RemoteLimitSwitchSource_FactoryDefaultOff: return "None (factory default value)";
            case RemoteLimitSwitchSource_RemoteTalonSRX : return "RemoteLimitSwitchSource_RemoteTalonSRX";
            case RemoteLimitSwitchSource_RemoteCANifier : return "RemoteLimitSwitchSource_RemoteCANifier";
            case RemoteLimitSwitchSource_Deactivated : return "RemoteLimitSwitchSource_Deactivated";
            default : return "InvalidValue";
        }

    }
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
