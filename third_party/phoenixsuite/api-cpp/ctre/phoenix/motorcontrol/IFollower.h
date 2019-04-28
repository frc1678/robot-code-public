#pragma once

namespace ctre {
namespace phoenix {
namespace motorcontrol {

/* forward proto */
class IMotorController;

/**
 * Interface for followers
 */
class IFollower {
public:
	virtual ~IFollower(){}
	/**
	 * Set the control mode and output value so that this motor controller will
	 * follow another motor controller. Currently supports following Victor SPX
	 * and Talon SRX.
	 *
	 * @param masterToFollow
	 *						Motor Controller object to follow.
	 */
	virtual void Follow(ctre::phoenix::motorcontrol::IMotorController & masterToFollow) = 0;
	/**
	 * When master makes a device, this routine is called to signal the update.
	 */
	virtual void ValueUpdated()= 0;
};

} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre
