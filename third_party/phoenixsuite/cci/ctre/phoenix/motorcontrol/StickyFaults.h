#pragma once
#include <sstream>
namespace ctre {
namespace phoenix {
namespace motorcontrol {

/**
 * All the sticky faults available to motor controllers
 */
struct StickyFaults {
	/**
	 * Motor Controller is under 6.5V
	 */
	bool UnderVoltage;
	/**
	 * Forward limit switch is tripped and device is trying to go forward
	 * Only trips when the device is limited
	 */
	bool ForwardLimitSwitch;
	/**
	 * Reverse limit switch is tripped and device is trying to go reverse
	 * Only trips when the device is limited
	 */
	bool ReverseLimitSwitch;
	/**
	 * Sensor is beyond forward soft limit and device is trying to go forward
	 * Only trips when the device is limited
	 */
	bool ForwardSoftLimit;
	/**
	 * Sensor is beyond reverse soft limit and device is trying to go reverse
	 * Only trips when the device is limited
	 */
	bool ReverseSoftLimit;
	/**
	 * Device was powered-on or reset while robot is enabled.
 	 * Check your breakers and wiring.
	 */
	bool ResetDuringEn;
	/**
	 * Device's sensor overflowed
	 */
	bool SensorOverflow;
	/**
	 * Device detects its sensor is out of phase
	 */
	bool SensorOutOfPhase;
	/**
	 * Not used, @see #ResetDuringEn
	 */
	bool HardwareESDReset;
	/**
	 * Remote Sensor is no longer detected on bus
	 */
	bool RemoteLossOfSignal;
	/**
	 * Device detects an API error
	 */
	bool APIError;

	/**
	 * @return true if any faults are tripped
	 */
	bool HasAnyFault() const {
		return 	UnderVoltage |
				ForwardLimitSwitch |
				ReverseLimitSwitch |
				ForwardSoftLimit |
				ReverseSoftLimit |
				ResetDuringEn |
				SensorOverflow |
				SensorOutOfPhase |
				HardwareESDReset |
				RemoteLossOfSignal |
				APIError;
	}
	/**
	 * @return Current fault list as a bit field
	 */
	int ToBitfield() const {
		int retval = 0;
		int mask = 1;
		retval |= UnderVoltage ? mask : 0; mask <<= 1;
		retval |= ForwardLimitSwitch ? mask : 0; mask <<= 1;
		retval |= ReverseLimitSwitch ? mask : 0; mask <<= 1;
		retval |= ForwardSoftLimit ? mask : 0; mask <<= 1;
		retval |= ReverseSoftLimit ? mask : 0; mask <<= 1;
		retval |= ResetDuringEn ? mask : 0; mask <<= 1;
		retval |= SensorOverflow ? mask : 0; mask <<= 1;
		retval |= SensorOutOfPhase ? mask : 0; mask <<= 1;
		retval |= HardwareESDReset ? mask : 0; mask <<= 1;
		retval |= RemoteLossOfSignal ? mask : 0; mask <<= 1;
		retval |= APIError ? mask : 0; mask <<= 1;
		return retval;
	}
	/**
	 * Creates fault list with specified bit field of faults
	 * 
	 * @param bits bit field of faults to update with
	 */
	StickyFaults(int bits) {
		int mask = 1;
		UnderVoltage = (bits & mask) ? true : false; mask <<= 1;
		ForwardLimitSwitch = (bits & mask) ? true : false; mask <<= 1;
		ReverseLimitSwitch = (bits & mask) ? true : false; mask <<= 1;
		ForwardSoftLimit = (bits & mask) ? true : false; mask <<= 1;
		ReverseSoftLimit = (bits & mask) ? true : false; mask <<= 1;
		ResetDuringEn = (bits & mask) ? true : false; mask <<= 1;
		SensorOverflow = (bits & mask) ? true : false; mask <<= 1;
		SensorOutOfPhase = (bits & mask) ? true : false; mask <<= 1;
		HardwareESDReset = (bits & mask) ? true : false; mask <<= 1;
		RemoteLossOfSignal = (bits & mask) ? true : false; mask <<= 1;
		APIError = (bits & mask) ? true : false; mask <<= 1;
	}
	StickyFaults() {
		UnderVoltage = false;
		ForwardLimitSwitch = false;
		ReverseLimitSwitch = false;
		ForwardSoftLimit = false;
		ReverseSoftLimit = false;
		ResetDuringEn = false;
		SensorOverflow = false;
		SensorOutOfPhase = false;
		HardwareESDReset = false;
		RemoteLossOfSignal = false;
		APIError = false;
	}
	/**
	 * @return string representation of current faults tripped
	 */
	std::string ToString() {
		std::stringstream work;
		work << " UnderVoltage:" << (UnderVoltage ? "1" : "0");
		work << " ForwardLimitSwitch:" << (ForwardLimitSwitch ? "1" : "0");
		work << " ReverseLimitSwitch:" << (ReverseLimitSwitch ? "1" : "0");
		work << " ForwardSoftLimit:" << (ForwardSoftLimit ? "1" : "0");
		work << " ReverseSoftLimit:" << (ReverseSoftLimit ? "1" : "0");
		work << " ResetDuringEn:" << (ResetDuringEn ? "1" : "0");
		work << " SensorOverflow:" << (SensorOverflow ? "1" : "0");
		work << " SensorOutOfPhase:" << (SensorOutOfPhase ? "1" : "0");
		work << " HardwareESDReset:" << (HardwareESDReset ? "1" : "0");
		work << " RemoteLossOfSignal:" << (RemoteLossOfSignal ? "1" : "0");
		work << " APIError:" << (APIError ? "1" : "0");
		return work.str();
	}
};

} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre
