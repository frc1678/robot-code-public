#pragma once

#include "ctre/phoenix/ErrorCode.h"
#include <string>

namespace ctre {
namespace phoenix {
namespace motorcontrol {

/**
 * Choose the feedback device for a motor controller
 */
enum FeedbackDevice {
    //NOTE: None was removed as it doesn't exist in firmware
    //TODO: Add None to firmware and add None back in
	/**
	 * Quadrature encoder
	 */
	QuadEncoder = 0,
	//1
	/**
	 * Analog potentiometer/encoder
	 */
	Analog = 2,
	//3
	/**
	 * Tachometer
	 */
	Tachometer = 4,
	/**
	 * CTRE Mag Encoder in Relative mode or
	 * any other device that uses PWM to encode its output
	 */
	PulseWidthEncodedPosition = 8,

	/**
	 * Sum0 + Sum1
	 */
	SensorSum = 9,
	/**
	 * Diff0 - Diff1
	 */
	SensorDifference = 10,
	/**
	 * Sensor configured in RemoteFilter0
	 */
	RemoteSensor0 = 11,
	/**
	 * Sensor configured in RemoteFilter1
	 */
	RemoteSensor1 = 12,
	//13
	//14
	/**
	 * Motor Controller will fake a sensor based on applied motor output.
	 */
	SoftwareEmulatedSensor  = 15,

	/**
	 * CTR mag encoder configured in absolute, is the same 
	 * as a PWM sensor.
	 */
	CTRE_MagEncoder_Absolute = PulseWidthEncodedPosition,
	/**
	 * CTR mag encoder configured in relative, is the same 
	 * as an quadrature encoder sensor.
	 */
	CTRE_MagEncoder_Relative = QuadEncoder,
};

/**
 * Choose the remote feedback device for a motor controller
 */
enum RemoteFeedbackDevice  {
    //NOTE: RemoteFeedbackDevice_None was removed as it doesn't exist in firmware
    //TODO: Add RemoteFeedbackDevice_None to firmware and add RemoteFeedbackDevice_None back in
    /**
     * Factory default setting for non-enhanced motor controllers
     */
	RemoteFeedbackDevice_FactoryDefaultOff = 0,
	/**
	 * Use Sum0 + Sum1
	 */
	RemoteFeedbackDevice_SensorSum = 9,
	/**
	 * Use Diff0 - Diff1
	 */
	RemoteFeedbackDevice_SensorDifference = 10,
	/**
	 * Use the sensor configured
	 * in filter0
	 */
	RemoteFeedbackDevice_RemoteSensor0 = 11,
	/**
	 * Use the sensor configured
	 * in filter1
	 */
	RemoteFeedbackDevice_RemoteSensor1 = 12,
	//13
	//14
	/**
	 * Motor Controller will fake a sensor based on applied motor output.
	 */
	RemoteFeedbackDevice_SoftwareEmulatedSensor = 15,
};

/**
 * Class to handle feedback device routines
 */
class FeedbackDeviceRoutines {
public:
	/**
     * Gets the string representation of selected feedback device
     * @param value feedback device to get string representation of
	 * @return String representation of selected feedback device
	 */
    static std::string toString(FeedbackDevice value) {
        switch(value) {
            case QuadEncoder : return "QuadEncoder";
            case Analog : return "Analog";
            case Tachometer : return "Tachometer";
            case PulseWidthEncodedPosition : return "PulseWidthEncodedPosition";
            case SensorSum : return "SensorSum";
            case SensorDifference : return "SensorDifference";
            case RemoteSensor0 : return "RemoteSensor0";
            case RemoteSensor1 : return "RemoteSensor1";
            case SoftwareEmulatedSensor : return "SoftwareEmulatedSensor";
            default : return "InvalidValue";

        }

    }

	/**
     * Gets the string representation of selected remote feedback device
     * @param value remote feedback device to get string representation of
	 * @return String representation of selected remote feedback device
	 */
    static std::string toString(RemoteFeedbackDevice value) {
        switch(value) {
            case RemoteFeedbackDevice_FactoryDefaultOff: return "None (factory default value)";
            case RemoteFeedbackDevice_SensorSum : return "RemoteFeedbackDevice_SensorSum";
            case RemoteFeedbackDevice_SensorDifference : return "RemoteFeedbackDevice_SensorDifference";
            case RemoteFeedbackDevice_RemoteSensor0 : return "RemoteFeedbackDevice_RemoteSensor0";
            case RemoteFeedbackDevice_RemoteSensor1 : return "RemoteFeedbackDevice_RemoteSensor1";
            case RemoteFeedbackDevice_SoftwareEmulatedSensor : return "RemoteFeedbackDevice_SoftwareEmulatedSensor";
            default : return "InvalidValue";
        }

    }
};
} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre
