#pragma once

namespace ctre {
namespace phoenix {
namespace motorcontrol {

/**
 * Choose the type of follower
 */
enum FollowerType {
	/**
	 * Follow the percentOutput the master is using
	 */
	FollowerType_PercentOutput = 0,
	/**
	 * Follow the auxiliary output the master is
	 * calculating. Used for 2-axis control.
	 * This typically means apply PID0 - PID1 from master.
	 */
	FollowerType_AuxOutput1,
};

} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre
