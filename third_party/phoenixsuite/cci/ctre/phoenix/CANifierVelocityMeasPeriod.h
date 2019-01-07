#pragma once

#include <string>

namespace ctre {
namespace phoenix {

enum CANifierVelocityMeasPeriod {
	Period_1Ms = 1,
	Period_2Ms = 2,
	Period_5Ms = 5,
	Period_10Ms = 10,
	Period_20Ms = 20,
	Period_25Ms = 25,
	Period_50Ms = 50,
	Period_100Ms = 100,
};

class CANifierVelocityMeasPeriodRoutines {
public:
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
