#pragma once

#include <string>
namespace ctre {
	namespace phoenix {
		namespace motorcontrol {

			/**
			 * Choose the invert type of the motor controller.
			 * None is the equivalent of SetInverted(false), where positive request yields positive voltage on M+.
			 * InvertMotorOutput is the equivelant of SetInverted(true), where positive request yields positive voltage on M-.
			 * FollowMaster/OpposeMaster will match/oppose a master Talon/Victor.  This requires device to be configured as a follower.
			 */
			enum class InvertType {
				None = 0, // Same as SetInverted(false)
				InvertMotorOutput = 1, // Same as SetInverted(true)
				FollowMaster = 2, // Follow the invert of the master this MC is following.
				OpposeMaster = 3, // Opposite of the invert of the master this MC is following.
			};

		} // namespace motorcontrol
	} // namespace phoenix
} // namespace ctre
