#pragma once

namespace ctre {
namespace phoenix {
namespace motorcontrol {

/**
 * Choose the neutral mode for a motor controller
 */
enum NeutralMode {
	/** Use the NeutralMode that is set in the MC's persistent storage. */
	EEPROMSetting = 0,
	/** When commanded to neutral, motor leads are set to high-impedance, allowing mechanism to coast. */
    Coast = 1,
	/** When commanded to neutral, motor leads are commonized electrically to reduce motion. */
    Brake = 2,
};

} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre
