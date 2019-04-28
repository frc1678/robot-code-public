#pragma once

namespace ctre {
namespace phoenix {
namespace sensors {

/**
 * Sticky faults available to Pigeon
 */
struct PigeonIMU_StickyFaults {
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
	 * Creates fault list with specified bit field of faults
	 * 
	 * @param bits bit field of faults to update with
	 */
	PigeonIMU_StickyFaults(int bits) {
		(void)bits;
	}
	PigeonIMU_StickyFaults() {
	}
};

} // sensors
} // phoenix
} // ctre
