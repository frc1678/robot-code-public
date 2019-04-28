#pragma once

namespace ctre {
namespace phoenix {

/**
 * Sticky Faults for CANifier (Currently has none)
 */
struct CANifierStickyFaults {
	/**
	 * @return true if any faults are tripped
	 */
	bool HasAnyFault() const {
		return false;
	}
	/**
	 * @return Current fault list as a bit field
	 */
	int ToBitfield() const {
		int retval = 0;
		return retval;
	}
	/**
	 * Updates current fault list with specified bit field of faults
	 * 
	 * @param bits bit field of faults to update with
	 */
	CANifierStickyFaults(int bits) {
		(void)bits;
	}
	CANifierStickyFaults() {
	}
};

} // phoenix
} // ctre
