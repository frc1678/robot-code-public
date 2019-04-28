#pragma once

#include <string>

namespace ctre {
namespace phoenix {
namespace motorcontrol {

/**
 * Velocity Measurement Periods
 */
enum VelocityMeasPeriod {
	/**
	 * 1ms measurement period
	 */
	Period_1Ms = 1,
	/**
	 * 2ms measurement period
	 */
	Period_2Ms = 2,
	/**
	 * 5ms measurement period
	 */
	Period_5Ms = 5,
	/**
	 * 10ms measurement period
	 */
	Period_10Ms = 10,
	/**
	 * 20ms measurement period
	 */
	Period_20Ms = 20,
	/**
	 * 25ms measurement period
	 */
	Period_25Ms = 25,
	/**
	 * 50ms measurement period
	 */
	Period_50Ms = 50,
	/**
	 * 100ms measurement period
	 */
	Period_100Ms = 100,
};
/**
 * Class to handle routines specific to VelocityMeasPeriod
 */
class VelocityMeasPeriodRoutines {
public:
    /**
     * String representation of specified VelocityMeasPeriod
     * @param value VelocityMeasPeriod to convert to a string
     * @return string representation of VelocityMeasPeriod
     */
    static std::string toString(VelocityMeasPeriod value) {
        switch(value) {
            case VelocityMeasPeriod::Period_1Ms : return "VelocityMeasPeriod::Period_1Ms";
            case VelocityMeasPeriod::Period_2Ms : return "VelocityMeasPeriod::Period_2Ms";
            case VelocityMeasPeriod::Period_5Ms : return "VelocityMeasPeriod::Period_5Ms";
            case VelocityMeasPeriod::Period_10Ms : return "VelocityMeasPeriod::Period_10Ms";
            case VelocityMeasPeriod::Period_20Ms : return "VelocityMeasPeriod::Period_20Ms";
            case VelocityMeasPeriod::Period_25Ms : return "VelocityMeasPeriod::Period_25Ms";
            case VelocityMeasPeriod::Period_50Ms : return "VelocityMeasPeriod::Period_50Ms";
            case VelocityMeasPeriod::Period_100Ms : return "VelocityMeasPeriod::Period_100Ms";
            default : return "InvalidValue";
        }
    }
};
} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre
