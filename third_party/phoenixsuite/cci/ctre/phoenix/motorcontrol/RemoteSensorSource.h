#pragma once

#include <string>

namespace ctre {
namespace phoenix {
namespace motorcontrol {

enum class RemoteSensorSource {
	RemoteSensorSource_Off,
	RemoteSensorSource_TalonSRX_SelectedSensor,
	RemoteSensorSource_Pigeon_Yaw,
	RemoteSensorSource_Pigeon_Pitch,
	RemoteSensorSource_Pigeon_Roll,
	RemoteSensorSource_CANifier_Quadrature,
	RemoteSensorSource_CANifier_PWMInput0,
	RemoteSensorSource_CANifier_PWMInput1,
	RemoteSensorSource_CANifier_PWMInput2,
	RemoteSensorSource_CANifier_PWMInput3,
	RemoteSensorSource_GadgeteerPigeon_Yaw,
	RemoteSensorSource_GadgeteerPigeon_Pitch,
	RemoteSensorSource_GadgeteerPigeon_Roll,
};
class RemoteSensorSourceRoutines {
public:
    static std::string toString(RemoteSensorSource value) {
        switch(value) {
            case RemoteSensorSource::RemoteSensorSource_Off                     : return "RemoteSensorSource::RemoteSensorSource_Off";
            case RemoteSensorSource::RemoteSensorSource_TalonSRX_SelectedSensor : return "RemoteSensorSource::RemoteSensorSource_TalonSRX_SelectedSensor";
            case RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw              : return "RemoteSensorSource::RemoteSensorSource_Pigeon_Yaw";
            case RemoteSensorSource::RemoteSensorSource_Pigeon_Pitch            : return "RemoteSensorSource::RemoteSensorSource_Pigeon_Pitch";
            case RemoteSensorSource::RemoteSensorSource_Pigeon_Roll             : return "RemoteSensorSource::RemoteSensorSource_Pigeon_Roll";
            case RemoteSensorSource::RemoteSensorSource_CANifier_Quadrature     : return "RemoteSensorSource::RemoteSensorSource_CANifier_Quadrature";
            case RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput0      : return "RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput0";
            case RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput1      : return "RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput1";
            case RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput2      : return "RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput2";
            case RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput3      : return "RemoteSensorSource::RemoteSensorSource_CANifier_PWMInput3";
            case RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Yaw     : return "RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Yaw";
            case RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Pitch   : return "RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Pitch";
            case RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Roll    : return "RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Roll";
            default : return "InvalidValue"; 
        }
    }
};
}
}
}
