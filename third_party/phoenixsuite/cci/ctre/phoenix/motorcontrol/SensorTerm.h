#pragma once

namespace ctre {
namespace phoenix {
namespace motorcontrol {

/**
 * Choose the sensor term for a motor controller
 */
enum class SensorTerm {
	/**
	 * Choose Sum0 for a term
	 */
	SensorTerm_Sum0,
	/**
	 * Choose Sum1 for a term
	 */
	SensorTerm_Sum1,
	/**
	 * Choose Diff0 for a term
	 */
	SensorTerm_Diff0,
	/**
	 * Choose Diff1 for a term
	 */
	SensorTerm_Diff1,
};

/**
 * Class to handle routines specific to SensorTerm
 */
class SensorTermRoutines {
public:
    /**
     * Gets the string representation of specified SensorTerm value
     * @param value SensorTerm to get string representation of
     * @return string representation of specified SensorTerm
     */
    static std::string toString(SensorTerm value) {
        switch(value) {
            case SensorTerm::SensorTerm_Sum0 : return "SensorTerm::SensorTerm_Sum0";
            case SensorTerm::SensorTerm_Sum1 : return "SensorTerm::SensorTerm_Sum1";
            case SensorTerm::SensorTerm_Diff0 : return "SensorTerm::SensorTerm_Diff0";
            case SensorTerm::SensorTerm_Diff1 : return "SensorTerm::SensorTerm_Diff1";
            default : return "InvalidValue"; 
        }
    }
};

}
}
}
