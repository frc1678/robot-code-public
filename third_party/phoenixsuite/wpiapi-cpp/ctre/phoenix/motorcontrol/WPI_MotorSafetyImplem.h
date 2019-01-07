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

class WPI_MotorSafetyImplem: public frc::MotorSafety {
public:
	WPI_MotorSafetyImplem(ctre::phoenix::motorcontrol::can::BaseMotorController * speedController, const std::string & description)
	{
		_speedController = speedController;
		_description = description;
	}
	/**
	 * Common interface to stop the motor until Set is called again.
	 */
	virtual void StopMotor() {
		_speedController->NeutralOutput();
	}

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
