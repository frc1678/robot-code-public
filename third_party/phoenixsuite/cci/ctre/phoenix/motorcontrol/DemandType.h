#pragma once

namespace ctre {
namespace phoenix {
namespace motorcontrol {
/**
 * How to interpret a demand value.
 */
enum DemandType {
	/**
	 * Ignore the demand value and apply neutral/no-change.
	 */
	DemandType_Neutral = 0,
	/**
	 * When closed-looping, set the target of the aux PID loop to the demand value.
	 *
	 * When following, follow the processed output of the combined 
	 * primary/aux PID output of the master.  The demand value is ignored.
	 * Although it is much cleaner to use the 2-param Follow() in such cases.
	 */
	DemandType_AuxPID = 1, //!< Target value of Aux PID loop 1.
	/**
	 * When closed-looping, add demand arbitrarily to the closed-loop output.
	 */
	DemandType_ArbitraryFeedForward = 2, //!< Simply add to the output
};

} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre
