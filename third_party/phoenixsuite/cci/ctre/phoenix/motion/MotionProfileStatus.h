#pragma once

#include "ctre/phoenix/motion/SetValueMotionProfile.h"
#include "ctre/phoenix/motion/TrajectoryPoint.h"

namespace ctre {
namespace phoenix {
namespace motion {

/**
 * Motion Profile Status
 * This is simply a data transer object.
 */
struct MotionProfileStatus {
	/**
	 * The available empty slots in the trajectory buffer.
	 *
	 * The robot API holds a "top buffer" of trajectory points, so your applicaion
	 * can dump several points at once.  The API will then stream them into the Talon's
	 * low-level buffer, allowing the Talon to act on them.
	 */
	size_t topBufferRem;
	/**
	 * The number of points in the top trajectory buffer.
	 */
	size_t topBufferCnt;
	/**
	 * The number of points in the low level Talon/Victor buffer.
	 */
	int btmBufferCnt;
	/**
	 * Set if isUnderrun ever gets set.
	 * Can be manually cleared by ClearMotionProfileHasUnderrun() or automatically cleared by StartMotionProfile().
	 * @see clearMotionProfileHasUnderrun()
	 */
	bool hasUnderrun;
	/**
	 * This is set if Talon/Victor needs to shift a point from its buffer into
	 * the active trajectory point however the buffer is empty. This gets cleared
	 * automatically when is resolved.
	 */
	bool isUnderrun;
	/**
	 * True if the active trajectory point is not empty, false otherwise.
	 * The members in activePoint are only valid if this signal is set.
	 */
	bool activePointValid;

	/**
	 * True if the active trajectory point is the last point of the profile
	 */
	bool isLast;

	/**
	 * The selected PID[0] profile slot of current profile
	 */
	int profileSlotSelect0;

	/**
	 * The selected auxiliary PID[1] profile slot of current profile
	 */
	int profileSlotSelect1;

	/**
	 * The current output mode of the motion profile executer (disabled, enabled, or hold).
	 * When changing the set() value in MP mode, it's important to check this signal to
	 * confirm the change takes effect before interacting with the top buffer.
	 */
	ctre::phoenix::motion::SetValueMotionProfile outputEnable;

	/** The applied duration of the active trajectory point */
	int timeDurMs;
};

} // namespace motion
} // namespace phoenix
} // namespace ctre

