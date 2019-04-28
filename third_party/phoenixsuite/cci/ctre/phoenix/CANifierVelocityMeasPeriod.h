#pragma once

#include <string>

namespace ctre {
namespace phoenix {

/** 
* Enum for velocity periods used for CANifier 
*/ 
enum CANifierVelocityMeasPeriod {
	/**
	 * 1ms velocity measurement period
	 */
	Period_1Ms = 1,
	/**
	 * 2ms velocity measurement period
	 */
	Period_2Ms = 2,
	/**
	 * 5ms velocity measurement period
	 */
	Period_5Ms = 5,
	/**
	 * 10ms velocity measurement period
	 */
	Period_10Ms = 10,
	/**
	 * 20ms velocity measurement period
	 */
	Period_20Ms = 20,
	/**
	 * 25ms velocity measurement period
	 */
	Period_25Ms = 25,
	/**
	 * 50ms velocity measurement period
	 */
	Period_50Ms = 50,
	/**
	 * 100ms velocity measurement period
	 */
	Period_100Ms = 100,
};

/**
 * Class to handle routines specific to VelocityMeasPeriod
 */
class CANifierVelocityMeasPeriodRoutines {
public:
    /**
     * String representation of specified CANifierVelocityMeasPeriod
     * @param value CANifierVelocityMeasPeriod to convert to a string
     * @return string representation of CANifierVelocityMeasPeriod
     */
    static std::string toString(CANifierVelocityMeasPeriod value) {
        switch(value) {
            case CANifierVelocityMeasPeriod::Period_1Ms : return "CANifierVelocityMeasPeriod::Period_1Ms";
            case CANifierVelocityMeasPeriod::Period_2Ms : return "CANifierVelocityMeasPeriod::Period_2Ms";
            case CANifierVelocityMeasPeriod::Period_5Ms : return "CANifierVelocityMeasPeriod::Period_5Ms";
            case CANifierVelocityMeasPeriod::Period_10Ms : return "CANifierVelocityMeasPeriod::Period_10Ms";
            case CANifierVelocityMeasPeriod::Period_20Ms : return "CANifierVelocityMeasPeriod::Period_20Ms";
            case CANifierVelocityMeasPeriod::Period_25Ms : return "CANifierVelocityMeasPeriod::Period_25Ms";
            case CANifierVelocityMeasPeriod::Period_50Ms : return "CANifierVelocityMeasPeriod::Period_50Ms";
            case CANifierVelocityMeasPeriod::Period_100Ms : return "CANifierVelocityMeasPeriod::Period_100Ms";
            default : return "InvalidValue";
        }
    }
};

} // namespace phoenix
} // namespace ctre
