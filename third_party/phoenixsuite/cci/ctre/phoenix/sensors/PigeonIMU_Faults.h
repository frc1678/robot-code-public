#pragma once

namespace ctre {
namespace phoenix {
namespace sensors {

/**
 * Faults available to Pigeon (Currently has none)
 */
struct PigeonIMU_Faults {
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
	PigeonIMU_Faults(int bits) {
		(void)bits;
	}
	PigeonIMU_Faults() {
	}
};

} // sensors
} // phoenix
} // ctre

