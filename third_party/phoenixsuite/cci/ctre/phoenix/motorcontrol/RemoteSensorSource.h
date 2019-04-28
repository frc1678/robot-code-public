#pragma once

#include <string>

namespace ctre {
namespace phoenix {
namespace motorcontrol {

/**
 * Choose the remote sensor source for a motor controller 
 */
enum class RemoteSensorSource {
	/**
	 * Don't use a sensor
	 */
	RemoteSensorSource_Off,
	/**
	 * Use a sensor connected to
	 * a TalonSRX and configured on
	 * the TalonSRX
	 */
	RemoteSensorSource_TalonSRX_SelectedSensor,
	/**
	 * Use a CAN Pigeon's Yaw value
	 */
	RemoteSensorSource_Pigeon_Yaw,
	/**
	 * Use a CAN Pigeon's Pitch value
	 */
	RemoteSensorSource_Pigeon_Pitch,
	/**
	 * Use a CAN Pigeon's Roll value
	 */
	RemoteSensorSource_Pigeon_Roll,
	/**
	 * Use a quadrature sensor
	 * connected to a CANifier
	 */
	RemoteSensorSource_CANifier_Quadrature,
	/**
	 * Use a PWM sensor connected
	 * to CANifier's PWM0
	 */
	RemoteSensorSource_CANifier_PWMInput0,
	/**
	 * Use a PWM sensor connected
	 * to CANifier's PWM1
	 */
	RemoteSensorSource_CANifier_PWMInput1,
	/**
	 * Use a PWM sensor connected
	 * to CANifier's PWM2
	 */
	RemoteSensorSource_CANifier_PWMInput2,
	/**
	 * Use a PWM sensor connected
	 * to CANifier's PWM3
	 */
	RemoteSensorSource_CANifier_PWMInput3,
	/**
	 * Use the yaw value of a pigeon
	 * connected to a talon over ribbon cable
	 */
	RemoteSensorSource_GadgeteerPigeon_Yaw,
	/**
	 * Use the pitch value of a pigeon
	 * connected to a talon over ribbon cable
	 */
	RemoteSensorSource_GadgeteerPigeon_Pitch,
	/**
	 * Use the roll value of a pigeon
	 * connected to a talon over ribbon cable
	 */
	RemoteSensorSource_GadgeteerPigeon_Roll,
};
/**
 * Class used to get string representation of a remote sensor source
 */
class RemoteSensorSourceRoutines {
public:
    /**
     * Get string representation of specified remote sensor source
     * @param value remote sensor source to get string of
     * @return string representation of specified remote sensor source
     */
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
