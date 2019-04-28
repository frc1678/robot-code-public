#pragma once

namespace ctre {
namespace phoenix {
namespace motorcontrol {

/**
 * The different status frames available to enhanced motor controllers
 */
enum StatusFrameEnhanced {
	/** 
	 * General Status
	 */
	Status_1_General = 0x1400,
	/**
	 * Feedback for selected sensor on primary PID[0].
	 */
	Status_2_Feedback0 = 0x1440,
	/**
	 * Analog sensor, motor controller 
	 * temperature, and voltage at input leads
	 */
	Status_4_AinTempVbat = 0x14C0,
	/**
	 * Miscellaneous signals
	 */
	Status_6_Misc = 0x1540,
	/**
	 * Communication status
	 */
	Status_7_CommStatus = 0x1580,
	/**
	 * Motion profile buffer status
	 */
	Status_9_MotProfBuffer = 0x1600,
	/**
	 * Old name for Status 10.
	 * Use @see #Status_10_Targets instead.
	 */
	Status_10_MotionMagic = 0x1640,
	/**
	 * Correct name for Status 10.
	 * Functionally equivalent to @see #Status_10_MotionMagic
	 */
	Status_10_Targets = 0x1640,
	/**
	 * Feedback for selected sensor on aux PID[1].
	 */
	Status_12_Feedback1 = 0x16C0,
	/**
	 * Primary PID
	 */
	Status_13_Base_PIDF0 = 0x1700,
	/**
	 * Auxiliary PID
	 */
	Status_14_Turn_PIDF1 = 0x1740,
	/**
	 * Firmware & API status information
	 */
	Status_15_FirmareApiStatus = 0x1780,
	/** 
	 * MotionProfile Targets for Auxiliary PID1. 
	 */
	Status_17_Targets1 = 0x1C00,

	/**
	 * Quadrature sensor 
	 */
	Status_3_Quadrature = 0x1480,
	/**
	 * Pulse width sensor
	 */
	Status_8_PulseWidth = 0x15C0,
	/**
	 * Gadgeteer status
	 */
	Status_11_UartGadgeteer = 0x1680,
};

/**
 * The different status frames available to motor controllers
 */
enum StatusFrame {
	/** 
	 * General Status
	 */
	Status_1_General_ = 0x1400,
	/**
	 * Main controller feedback
	 */
	Status_2_Feedback0_ = 0x1440,
	/**
	 * Analog sensor, motor controller 
	 * temperature, and voltage at input leads
	 */
	Status_4_AinTempVbat_ = 0x14C0,
	/**
	 * Miscellaneous signals
	 */
	Status_6_Misc_ = 0x1540,
	/**
	 * Communication status to controller
	 */
	Status_7_CommStatus_ = 0x1580,
	/**
	 * Motion profile buffer status
	 */
	Status_9_MotProfBuffer_ = 0x1600,
	/**
	 * Old name for Status 10.
	 * Use @see #Status_10_Targets instead.
	 */
	Status_10_MotionMagic_ = 0x1640,
	/**
	 * Correct name for Status 10.
	 * Functionally equivalent to @see #Status_10_MotionMagic
	 */
	Status_10_Targets_ = 0x1640,
	/**
	 * Secondary controller feedback
	 */
	Status_12_Feedback1_ = 0x16C0,
	/**
	 * Base PID
	 */
	Status_13_Base_PIDF0_ = 0x1700,
	/**
	 * Auxiliary PID
	 */
	Status_14_Turn_PIDF1_ = 0x1740,
	/**
	 * Firmware & API status information
	 */
	Status_15_FirmareApiStatus_ = 0x1780,
	/** 
	 * MotionProfile Targets for Auxiliary PID1. 
	 */
	Status_17_Targets1_ = 0x1C00,
};

/**
 * Class to allow conversion from StatusFrame to EnhancedStatusFrame
 */
class StatusFrameRoutines {
public:
	/**
	 * Converts a status frame to an enhanced status frame
	 * @param statusFrame frame to convert
	 * @return enhanced status frame version of statusFrame
	 */
	StatusFrameEnhanced Promote(StatusFrame statusFrame) {
		return (StatusFrameEnhanced) statusFrame;
	}
};
} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre
