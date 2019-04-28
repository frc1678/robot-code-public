/**
 * Wrapper for WPI Motor Safety Implem. This also allows late/lazy
 * construction of WPI's motor safety object (which mitigates late-released bugs from WPI).
 */
#pragma once

//Need to disable certain warnings for WPI headers.
#if __GNUC__
	#pragma GCC diagnostic push
	#pragma GCC diagnostic ignored "-Wconversion"
#elif _MSC_VER
	#pragma warning( push )
	#pragma warning( disable : 4522 4458 4522)
#endif

#include "ctre/phoenix/motorcontrol/can/BaseMotorController.h"
#include "frc/MotorSafety.h"
#include "wpi/raw_ostream.h"

//Put the warning settings back to normal
#if __GNUC__
	#pragma GCC diagnostic pop
#elif _MSC_VER
	#pragma warning( pop )
#endif

namespace ctre {
namespace phoenix {
namespace motorcontrol {

/**
 * implem of MotorSafety interface in WPI. This also allows late/lazy
 * construction of WPI's motor safety object (which mitigates late-released bugs from WPI).
 */
class WPI_MotorSafetyImplem: public frc::MotorSafety {
public:
	/**
	 * Constructor for WPI_MotorSafetyImplem
	 * @param speedController Speed Controller to implement motor safety on
	 * @param description Description of speed controller
	 */
	WPI_MotorSafetyImplem(ctre::phoenix::motorcontrol::can::BaseMotorController * speedController, const std::string & description)
	{
		_speedController = speedController;
		_description = description;
	}
	/**
	 * Stop the controller
	 */
	virtual void StopMotor() {
		_speedController->NeutralOutput();
	}

	/**
	 * @return Description of speed controller
	 */
	virtual void GetDescription(wpi::raw_ostream& desc) const {
		desc << _description.c_str();
	}
	
private:
	ctre::phoenix::motorcontrol::can::BaseMotorController * _speedController;
	std::string _description;
};

} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre
