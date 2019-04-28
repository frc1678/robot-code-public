#pragma once

namespace ctre {
namespace phoenix {

/**
 * Faults available to CANifier (Currently has none)
 */
struct CANifierFaults {
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
	CANifierFaults(int bits) {
		(void)bits;
	}
	CANifierFaults() {
	}
};

} // phoenix
} // ctre

