/**
 * WPI Compliant motor controller class.
 * WPILIB's object model requires many interfaces to be implemented to use
 * the various features.
 * This includes...
 * - Software PID loops running in the robot controller
 * - LiveWindow/Test mode features
 * - Motor Safety (auto-turn off of motor if Set stops getting called)
 * - Single Parameter set that assumes a simple motor controller.
 */
#pragma once

#include "ctre/phoenix/motorcontrol/can/TalonSRX.h"

//Need to disable certain warnings for WPI headers.
#if __GNUC__
	#pragma GCC diagnostic push
	#pragma GCC diagnostic ignored "-Wconversion"
#elif _MSC_VER
	#pragma warning(push)
	#pragma warning(disable : 4522 4458 4522)
#endif

#include "frc/smartdashboard/SendableBase.h"
#include "frc/smartdashboard/SendableBuilder.h"
#include "frc/SpeedController.h"
#include "frc/MotorSafety.h"
#include "wpi/raw_ostream.h"
#include <mutex>

//Put the warning settings back to normal
#if __GNUC__
	#pragma GCC diagnostic pop
#elif _MSC_VER
	#pragma warning(pop)
#endif

namespace ctre {
namespace phoenix {
namespace motorcontrol {
namespace can {

/**
 * CTRE Talon SRX Motor Controller when used on CAN Bus.
 */
class WPI_TalonSRX : public virtual TalonSRX,
					 public virtual frc::SpeedController,
					 public frc::SendableBase
{
public:
	WPI_TalonSRX(int deviceNumber);
	virtual ~WPI_TalonSRX();

	WPI_TalonSRX() = delete;
	WPI_TalonSRX(WPI_TalonSRX const &) = delete;
	WPI_TalonSRX &operator=(WPI_TalonSRX const &) = delete;

	//----------------------- set/get routines for WPILIB interfaces -------------------//
	/**
	 * Common interface for setting the speed of a simple speed controller.
	 *
	 * @param speed The speed to set.  Value should be between -1.0 and 1.0.
	 * 									Value is also saved for Get().
	 */
	virtual void Set(double speed);
	virtual void PIDWrite(double output);

	/**
	 * Common interface for getting the current set speed of a speed controller.
	 *
	 * @return The current set speed.  Value is between -1.0 and 1.0.
	 */
	virtual double Get() const;

	//----------------------- Intercept CTRE calls for motor safety -------------------//
	/**
	 * Sets the appropriate output on the talon, depending on the mode.
	 * @param mode The output mode to apply.
	 * In PercentOutput, the output is between -1.0 and 1.0, with 0.0 as stopped.
	 * In Current mode, output value is in amperes.
	 * In Velocity mode, output value is in position change / 100ms.
	 * In Position mode, output value is in encoder ticks or an analog value,
	 *   depending on the sensor.
	 * In Follower mode, the output value is the integer device ID of the talon to
	 * duplicate.
	 *
	 * @param value The setpoint value, as described above.
	 *
	 *
	 *	Standard Driving Example:
	 *	_talonLeft.set(ControlMode.PercentOutput, leftJoy);
	 *	_talonRght.set(ControlMode.PercentOutput, rghtJoy);
	 */
	virtual void Set(ControlMode mode, double value);
	/**
     * @deprecated use 4 parameter set
	 * @param mode Sets the appropriate output on the talon, depending on the mode.
	 * @param demand0 The output value to apply.
	 * 	such as advanced feed forward and/or auxiliary close-looping in firmware.
	 * In PercentOutput, the output is between -1.0 and 1.0, with 0.0 as stopped.
	 * In Current mode, output value is in amperes.
	 * In Velocity mode, output value is in position change / 100ms.
	 * In Position mode, output value is in encoder ticks or an analog value,
	 *   depending on the sensor. See
	 * In Follower mode, the output value is the integer device ID of the talon to
	 * duplicate.
	 *
	 * @param demand1 Supplemental value.  This will also be control mode specific for future features.
	 */
	virtual void Set(ControlMode mode, double demand0, double demand1);
	/**
	 * @param mode Sets the appropriate output on the talon, depending on the mode.
	 * @param demand0 The output value to apply.
	 * 	such as advanced feed forward and/or auxiliary close-looping in firmware.
	 * In PercentOutput, the output is between -1.0 and 1.0, with 0.0 as stopped.
	 * In Current mode, output value is in amperes.
	 * In Velocity mode, output value is in position change / 100ms.
	 * In Position mode, output value is in encoder ticks or an analog value,
	 *   depending on the sensor. See
	 * In Follower mode, the output value is the integer device ID of the talon to
	 * duplicate.
	 *
	 * @param demand1Type The demand type for demand1.
	 * Neutral: Ignore demand1 and apply no change to the demand0 output.
	 * AuxPID: Use demand1 to set the target for the auxiliary PID 1.
	 * ArbitraryFeedForward: Use demand1 as an arbitrary additive value to the
	 *	 demand0 output.  In PercentOutput the demand0 output is the motor output,
	 *   and in closed-loop modes the demand0 output is the output of PID0.
	 * @param demand1 Supplmental output value.  Units match the set mode.
	 *
	 *
	 *  Arcade Drive Example:
	 *		_talonLeft.set(ControlMode.PercentOutput, joyForward, DemandType.ArbitraryFeedForward, +joyTurn);
	 *		_talonRght.set(ControlMode.PercentOutput, joyForward, DemandType.ArbitraryFeedForward, -joyTurn);
	 *
	 *	Drive Straight Example:
	 *	Note: Selected Sensor Configuration is necessary for both PID0 and PID1.
	 *		_talonLeft.follow(_talonRght, FollwerType.AuxOutput1);
	 *		_talonRght.set(ControlMode.PercentOutput, joyForward, DemandType.AuxPID, desiredRobotHeading);
	 *
	 *	Drive Straight to a Distance Example:
	 *	Note: Other configurations (sensor selection, PID gains, etc.) need to be set.
	 *		_talonLeft.follow(_talonRght, FollwerType.AuxOutput1);
	 *		_talonRght.set(ControlMode.MotionMagic, targetDistance, DemandType.AuxPID, desiredRobotHeading);
	 */
	virtual void Set(ControlMode mode, double demand0, DemandType demand1Type, double demand1);
	//----------------------- Invert routines -------------------//
	/**
	 * Common interface for inverting direction of a speed controller.
	 *
	 * @param isInverted The state of inversion, true is inverted.
	 */
	virtual void SetInverted(bool isInverted);
	/**
	 * Common interface for inverting direction of a speed controller.
	 *
	 * @param invertType The invert strategy to use. Follower controllers
	 * 					that mirror/oppose the master controller should 
	 *					use this method.
	 */
	virtual void SetInverted(InvertType invertType);
	/**
	 * Common interface for returning the inversion state of a speed controller.
	 *
	 * @return isInverted The state of inversion, true is inverted.
	 */
	virtual bool GetInverted() const;
	//----------------------- turn-motor-off routines-------------------//
	/**
	 * Common interface for disabling a motor.
	 */
	virtual void Disable();
	/**
	 * Common interface to stop the motor until Set is called again.
	 */
	virtual void StopMotor();

	void GetDescription(wpi::raw_ostream &desc) const;

	//----------------------- Motor Safety comparable interface ----------//
	/**
	 * Feed the motor safety object.
	 *
	 * Resets the timer on this object that is used to do the timeouts.
	 */
	void Feed();

	/**
	 * Set the expiration time for the corresponding motor safety object.
	 *
	 * @param expirationTime The timeout value in seconds.
	 */
	void SetExpiration(double expirationTime);

	/**
	 * Retrieve the timeout value for the corresponding motor safety object.
	 *
	 * @return the timeout value in seconds.
	 */
	double GetExpiration() const;

	/**
	 * Determine if the motor is still operating or has timed out.
	 *
	 * @return true if the motor is still operating normally and hasn't timed out.
	 */
	bool IsAlive() const;

	/**
	 * Enable/disable motor safety for this device.
	 *
	 * Turn on and off the motor safety option for this PWM object.
	 *
	 * @param enabled True if motor safety is enforced for this object.
	 */
	void SetSafetyEnabled(bool enabled);

	/**
	 * Return the state of the motor safety enabled flag.
	 *
	 * Return if the motor safety is currently enabled for this device.
	 *
	 * @return True if motor safety is enforced for this device.
	 */
	bool IsSafetyEnabled() const;

	/**
	 * @return the Motor Safety object corresponding to this device.
	 */
	frc::MotorSafety &GetMotorSafety();

protected:
	virtual void InitSendable(frc::SendableBuilder &builder);
private:
	double _speed = 0;
	std::string _desc;
	mutable std::mutex _lockMotorSaf;
	mutable frc::MotorSafety *_motorSafety = nullptr;
	double _motSafeExpiration = 0.1;
};

} // namespace can
} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre