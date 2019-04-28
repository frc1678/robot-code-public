#pragma once

#include "ctre/phoenix/motorcontrol/ControlMode.h"
#include "ctre/phoenix/motorcontrol/ControlFrame.h"
#include "ctre/phoenix/motorcontrol/DemandType.h"
#include "ctre/phoenix/motorcontrol/InvertType.h"
#include "ctre/phoenix/motorcontrol/NeutralMode.h"
#include "ctre/phoenix/motorcontrol/FeedbackDevice.h"
#include "ctre/phoenix/motorcontrol/RemoteSensorSource.h"
#include "ctre/phoenix/motorcontrol/SensorTerm.h"
#include "ctre/phoenix/motorcontrol/StatusFrame.h"
#include "ctre/phoenix/motorcontrol/LimitSwitchType.h"
#include "ctre/phoenix/motorcontrol/Faults.h"
#include "ctre/phoenix/motorcontrol/StickyFaults.h"
#include "ctre/phoenix/paramEnum.h"
#include "ctre/phoenix/motion/TrajectoryPoint.h"
#include "ctre/phoenix/motion/MotionProfileStatus.h"
#include "ctre/phoenix/ErrorCode.h"
#include "IFollower.h"

namespace ctre {
namespace phoenix {
namespace motorcontrol {

/**
 * Interface for motor controllers
 */
class IMotorController: public virtual IFollower {
public:
	virtual ~IMotorController() {
	}
	//------ Set output routines. ----------//
	/**
	 * Sets the appropriate output on the talon, depending on the mode.
	 * @param Mode The output mode to apply.
	 * In PercentOutput, the output is between -1.0 and 1.0, with 0.0 as stopped.
	 * In Current mode, output value is in amperes.
	 * In Velocity mode, output value is in position change / 100ms.
	 * In Position mode, output value is in encoder ticks or an analog value,
	 *   depending on the sensor.
	 * In Follower mode, the output value is the integer device ID of the talon to
	 * duplicate.
	 *
	 * @param demand The setpoint value, as described above.
	 *
	 *
	 *	Standard Driving Example:
	 *	_talonLeft.set(ControlMode.PercentOutput, leftJoy);
	 *	_talonRght.set(ControlMode.PercentOutput, rghtJoy);
	 */
	virtual void Set(ControlMode Mode, double demand) = 0;
	/**
     * @deprecated use 4 parameter set
	 * @param Mode Sets the appropriate output on the talon, depending on the mode.
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
	virtual void Set(ControlMode Mode, double demand0, double demand1) = 0;
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
	virtual void Set(ControlMode mode, double demand0, DemandType demand1Type, double demand1) = 0;
	/**
	 * Neutral the motor output by setting control mode to disabled.
	 */
	virtual void NeutralOutput() = 0;
	/**
	 * Sets the mode of operation during neutral throttle output.
	 *
	 * @param neutralMode
	 *            The desired mode of operation when the Controller output
	 *            throttle is neutral (ie brake/coast)
	 **/
	virtual void SetNeutralMode(NeutralMode neutralMode) = 0;

	//------ Invert behavior ----------//
	/**
	 * Sets the phase of the sensor. Use when controller forward/reverse output
	 * doesn't correlate to appropriate forward/reverse reading of sensor.
	 * Pick a value so that positive PercentOutput yields a positive change in sensor.
	 * After setting this, user can freely call SetInverted() with any value.
	 *
	 * @param PhaseSensor
	 *            Indicates whether to invert the phase of the sensor.
	 */
	virtual void SetSensorPhase(bool PhaseSensor) = 0;
	/**
	 * Inverts the hbridge output of the motor controller.
	 *
	 * This does not impact sensor phase and should not be used to correct sensor polarity.
	 *
	 * This will invert the hbridge output but NOT the LEDs.
	 * This ensures....
	 *  - Green LEDs always represents positive request from robot-controller/closed-looping mode.
	 *  - Green LEDs correlates to forward limit switch.
	 *  - Green LEDs correlates to forward soft limit.
	 *
	 * @param invert
	 *            Invert state to set.
	 */
	virtual void SetInverted(bool invert) = 0;
	/**
	 * Inverts the hbridge output of the motor controller in relation to the master if present 
	 *
	 * This does not impact sensor phase and should not be used to correct sensor polarity.
	 *
	 * This will allow you to either:
	 *  - Not invert the motor
	 *  - Invert the motor
	 *  - Always follow the master regardless of master's inversion
	 *  - Always oppose the master regardless of master's inversion
	 *
	 * @param invertType
	 *            Invert state to set.
	 */
	virtual void SetInverted(InvertType invertType) = 0;
	/**
	 * @return invert setting of motor output.
	 */
	virtual bool GetInverted() const = 0;

	//----- Factory Default Configuration -----//
	/**
	 * Revert all configurations to factory default values.
	 * Use this before your individual config* calls to avoid having to config every single param.
	 *
	 * Alternatively you can use the configAllSettings routine.
	 * 
	 * @param timeout
	 *            Timeout value in ms. Function will generate error if config is
	 *            not successful within timeout.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigFactoryDefault(int timeout) = 0;

	//----- general output shaping ------------------//
	/**
	 * Configures the open-loop ramp rate of throttle output.
	 *
	 * @param secondsFromNeutralToFull
	 *            Minimum desired time to go from neutral to full throttle. A
	 *            value of '0' will disable the ramp.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigOpenloopRamp(double secondsFromNeutralToFull,
			int timeoutMs = 0) = 0;
	/**
	 * Configures the closed-loop ramp rate of throttle output.
	 *
	 * @param secondsFromNeutralToFull
	 *            Minimum desired time to go from neutral to full throttle. A
	 *            value of '0' will disable the ramp.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigClosedloopRamp(double secondsFromNeutralToFull,
			int timeoutMs = 0) = 0;
	/**
	 * Configures the forward peak output percentage.
	 *
	 * @param percentOut
	 *            Desired peak output percentage. [0,1]
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigPeakOutputForward(double percentOut,
			int timeoutMs = 0) = 0;
	/**
	 * Configures the reverse peak output percentage.
	 *
	 * @param percentOut
	 *            Desired peak output percentage.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigPeakOutputReverse(double percentOut,
			int timeoutMs = 0) = 0;
	/**
	 * Configures the forward nominal output percentage.
	 *
	 * @param percentOut
	 *            Nominal (minimum) percent output. [0,+1]
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigNominalOutputForward(double percentOut,
			int timeoutMs = 0) = 0;
	/**
	 * Configures the reverse nominal output percentage.
	 *
	 * @param percentOut
	 *            Nominal (minimum) percent output. [-1,0]
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigNominalOutputReverse(double percentOut,
			int timeoutMs = 0) = 0;
	/**
	 * Configures the output deadband percentage.
	 *
	 * @param percentDeadband
	 *            Desired deadband percentage. Minimum is 0.1%, Maximum is 25%.
	 *            Pass 0.04 for 4% (factory default).
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigNeutralDeadband(double percentDeadband,
			int timeoutMs = 0) = 0;

	//------ Voltage Compensation ----------//
	/**
	 * Configures the Voltage Compensation saturation voltage.
	 *
	 * @param voltage
	 *            This is the max voltage to apply to the hbridge when voltage
	 *            compensation is enabled.  For example, if 10 (volts) is specified
	 *            and a TalonSRX is commanded to 0.5 (PercentOutput, closed-loop, etc)
	 *            then the TalonSRX will attempt to apply a duty-cycle to produce 5V.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigVoltageCompSaturation(double voltage,
			int timeoutMs = 0) = 0;
	/**
	 * Configures the voltage measurement filter.
	 *
	 * @param filterWindowSamples
	 *            Number of samples in the rolling average of voltage
	 *            measurement.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigVoltageMeasurementFilter(int filterWindowSamples,
			int timeoutMs = 0) = 0;
	/**
	 * Enables voltage compensation. If enabled, voltage compensation works in
	 * all control modes.
	 *
	 * Be sure to configure the saturation voltage before enabling this.
	 *
	 * @param enable
	 *            Enable state of voltage compensation.
	 **/
	virtual void EnableVoltageCompensation(bool enable) = 0;

	//------ General Status ----------//
	/**
	 * Gets the bus voltage seen by the device.
	 *
	 * @return The bus voltage value (in volts).
	 */
	virtual double GetBusVoltage() = 0;
	/**
	 * Gets the output percentage of the motor controller.
	 *
	 * @return Output of the motor controller (in percent).
	 */
	virtual double GetMotorOutputPercent() = 0;
	/**
	 * @return applied voltage to motor  in volts.
	 */
	virtual double GetMotorOutputVoltage() = 0;
	/**
	 * Gets the temperature of the motor controller.
	 *
	 * @return Temperature of the motor controller (in 'C)
	 */
	virtual double GetTemperature() = 0;

	//------ sensor selection ----------//
	/**
	 * Select the remote feedback device for the motor controller.
	 * Most CTRE CAN motor controllers will support remote sensors over CAN.
	 *
	 * @param feedbackDevice
	 *            Remote Feedback Device to select.
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigSelectedFeedbackSensor(
			RemoteFeedbackDevice feedbackDevice, int pidIdx = 0, int timeoutMs = 0) = 0;
	/**
	 * The Feedback Coefficient is a scalar applied to the value of the
	 * feedback sensor.  Useful when you need to scale your sensor values
	 * within the closed-loop calculations.  Default value is 1.
	 *
	 * Selected Feedback Sensor register in firmware is the decoded sensor value
	 * multiplied by the Feedback Coefficient.
	 *
	 * @param coefficient
	 *            Feedback Coefficient value.  Maximum value of 1.
	 *						Resolution is 1/(2^16).  Cannot be 0.
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigSelectedFeedbackCoefficient(
			double coefficient, int pidIdx = 0, int timeoutMs = 0) = 0;
	/**
	 * Select what remote device and signal to assign to Remote Sensor 0 or Remote Sensor 1.
	 * After binding a remote device and signal to Remote Sensor X, you may select Remote Sensor X
	 * as a PID source for closed-loop features.
	 *
	 * @param deviceID
 	 *            The device ID of the remote sensor device.
	 * @param remoteSensorSource
	 *            The remote sensor device and signal type to bind.
	 * @param remoteOrdinal
	 *            0 for configuring Remote Sensor 0,
	 *            1 for configuring Remote Sensor 1
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigRemoteFeedbackFilter(int deviceID,
			RemoteSensorSource remoteSensorSource, int remoteOrdinal,
			int timeoutMs = 0)= 0;
	/**
	 * Select what sensor term should be bound to switch feedback device.
	 * Sensor Sum = Sensor Sum Term 0 - Sensor Sum Term 1
	 * Sensor Difference = Sensor Diff Term 0 - Sensor Diff Term 1
	 * The four terms are specified with this routine.  Then Sensor Sum/Difference
	 * can be selected for closed-looping.
	 *
	 * @param sensorTerm Which sensor term to bind to a feedback source.
	 * @param feedbackDevice The sensor signal to attach to sensorTerm.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigSensorTerm(SensorTerm sensorTerm, FeedbackDevice feedbackDevice, int timeoutMs = 0)= 0;

	//------- sensor status --------- //
	/**
	 * Get the selected sensor position (in raw sensor units).
	 *
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop. See
	 *            Phoenix-Documentation for how to interpret.
	 *
	 * @return Position of selected sensor (in raw sensor units).
	 */
	virtual int GetSelectedSensorPosition(int pidIdx = 0) = 0;
	/**
	 * Get the selected sensor velocity.
	 *
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @return selected sensor (in raw sensor units) per 100ms.
	 * See Phoenix-Documentation for how to interpret.
	 */
	virtual int GetSelectedSensorVelocity(int pidIdx = 0) = 0;
	/**
	 * Sets the sensor position to the given value.
	 *
	 * @param sensorPos
	 *            Position to set for the selected sensor (in raw sensor units).
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode SetSelectedSensorPosition(int sensorPos, int pidIdx = 0,
			int timeoutMs = 50) = 0;

	//------ status frame period changes ----------//
	/**
	 * Sets the period of the given control frame.
	 *
	 * @param frame
	 *            Frame whose period is to be changed.
	 * @param periodMs
	 *            Period in ms for the given frame.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode SetControlFramePeriod(ControlFrame frame,
			int periodMs) = 0;
	/**
	 * Sets the period of the given status frame.
	 *
	 * User ensure CAN Bus utilization is not high.
	 *
	 * This setting is not persistent and is lost when device is reset. If this
	 * is a concern, calling application can use HasResetOccurred() to determine if the
	 * status frame needs to be reconfigured.
	 *
	 * @param frame
	 *            Frame whose period is to be changed.
	 * @param periodMs
	 *            Period in ms for the given frame.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode SetStatusFramePeriod(StatusFrame frame, uint8_t periodMs,
			int timeoutMs = 0) = 0;
	/**
	 * Gets the period of the given status frame.
	 *
	 * @param frame
	 *            Frame to get the period of.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Period of the given status frame.
	 */
	virtual int GetStatusFramePeriod(StatusFrame frame, int timeoutMs = 0) = 0;

	//----- velocity signal conditionaing ------//
	/* not supported */

	//------ remote limit switch ----------//
	/**
	 * Configures the forward limit switch for a remote source. For example, a
	 * CAN motor controller may need to monitor the Limit-F pin of another Talon
	 * or CANifier.
	 *
	 * @param type
	 *            Remote limit switch source. User can choose between a remote
	 *            Talon SRX, CANifier, or deactivate the feature.
	 * @param normalOpenOrClose
	 *            Setting for normally open, normally closed, or disabled. This
	 *            setting matches the Phoenix Tuner drop down.
	 * @param deviceID
	 *            Device ID of remote source (Talon SRX or CANifier device ID).
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigForwardLimitSwitchSource(
			RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
			int deviceID, int timeoutMs = 0) = 0;
	/**
	 * Configures the reverse limit switch for a remote source. For example, a
	 * CAN motor controller may need to monitor the Limit-R pin of another Talon
	 * or CANifier.
	 *
	 * @param type
	 *            Remote limit switch source. User can choose between a remote
	 *            Talon SRX, CANifier, or deactivate the feature.
	 * @param normalOpenOrClose
	 *            Setting for normally open, normally closed, or disabled. This
	 *            setting matches the Phoenix Tuner drop down.
	 * @param deviceID
	 *            Device ID of remote source (Talon SRX or CANifier device ID).
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigReverseLimitSwitchSource(
			RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
			int deviceID, int timeoutMs = 0) = 0;
	/**
	 * Sets the enable state for limit switches.
	 *
	 * @param enable
	 *            Enable state for limit switches.
	 **/
	virtual void OverrideLimitSwitchesEnable(bool enable) = 0;

	//------ local limit switch ----------//
	/* not supported */

	//------ soft limit ----------//
	/**
	 * Configures the forward soft limit threhold.
	 *
	 * @param forwardSensorLimit
	 *            Forward Sensor Position Limit (in raw sensor units).
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigForwardSoftLimitThreshold(int forwardSensorLimit,
			int timeoutMs = 0) = 0;
	/**
	 * Configures the reverse soft limit threshold.
	 *
	 * @param reverseSensorLimit
	 *            Reverse Sensor Position Limit (in raw sensor units).
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigReverseSoftLimitThreshold(int reverseSensorLimit,
			int timeoutMs = 0) = 0;
	/**
	 * Configures the forward soft limit enable.
	 *
	 * @param enable
	 *            Forward Sensor Position Limit Enable.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigForwardSoftLimitEnable(bool enable,
			int timeoutMs = 0) = 0;
	/**
	 * Configures the reverse soft limit enable.
	 *
	 * @param enable
	 *            Reverse Sensor Position Limit Enable.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigReverseSoftLimitEnable(bool enable,
			int timeoutMs = 0) = 0;
	/**
	 * Can be used to override-disable the soft limits.
	 * This function can be used to quickly disable soft limits without
	 * having to modify the persistent configuration.
	 *
	 * @param enable
	 *            Enable state for soft limit switches.
	 */
	virtual void OverrideSoftLimitsEnable(bool enable) = 0;

	//------ Current Lim ----------//
	/* not supported */

	//------ Config Close loop ----------//
	/**
	 * Sets the 'P' constant in the given parameter slot.
	 * This is multiplied by closed loop error in sensor units.
	 * Note the closed loop output interprets a final value of 1023 as full output.
	 * So use a gain of '0.25' to get full output if err is 4096u (Mag Encoder 1 rotation)
	 *
	 * @param slotIdx
	 *            Parameter slot for the constant.
	 * @param value
	 *            Value of the P constant.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode Config_kP(int slotIdx, double value, int timeoutMs = 0) = 0;
	/**
	 * Sets the 'I' constant in the given parameter slot.
	 * This is multiplied by accumulated closed loop error in sensor units every PID Loop.  
	 * Note the closed loop output interprets a final value of 1023 as full output.  
	 * So use a gain of '0.00025' to get full output if err is 4096u for 1000 loops (accumulater holds 4,096,000),
	 * [which is equivalent to one CTRE mag encoder rotation for 1000 milliseconds].
	 *
	 * @param slotIdx
	 *            Parameter slot for the constant.
	 * @param value
	 *            Value of the I constant.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode Config_kI(int slotIdx, double value, int timeoutMs = 0) = 0;
	/**
	 * Sets the 'D' constant in the given parameter slot.
	 *
	 * This is multiplied by derivative error (sensor units per PID loop, typically 1ms).
	 * Note the closed loop output interprets a final value of 1023 as full output.
	 * So use a gain of '250' to get full output if derr is 4096u (Mag Encoder 1 rotation) per 1000 loops (typ 1 sec)
	 *
	 * @param slotIdx
	 *            Parameter slot for the constant.
	 * @param value
	 *            Value of the D constant.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode Config_kD(int slotIdx, double value, int timeoutMs = 0) = 0;
	/**
	 * Sets the 'F' constant in the given parameter slot.
	 *
	 * See documentation for calculation details.
	 * If using velocity, motion magic, or motion profile,
	 * use (1023 * duty-cycle / sensor-velocity-sensor-units-per-100ms).
	 *
	 * @param slotIdx
	 *            Parameter slot for the constant.
	 * @param value
	 *            Value of the F constant.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode Config_kF(int slotIdx, double value, int timeoutMs = 0) = 0;
	/**
	 * Sets the Integral Zone constant in the given parameter slot. If the
	 * (absolute) closed-loop error is outside of this zone, integral
	 * accumulator is automatically cleared. This ensures than integral wind up
	 * events will stop after the sensor gets far enough from its target.
	 *
	 * @param slotIdx
	 *            Parameter slot for the constant.
	 * @param izone
	 *            Value of the Integral Zone constant (closed loop error units X
	 *            1ms).
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode Config_IntegralZone(int slotIdx, int izone,
			int timeoutMs = 0) = 0;
	/**
	 * Sets the allowable closed-loop error in the given parameter slot.
	 *
	 * @param slotIdx
	 *            Parameter slot for the constant.
	 * @param allowableCloseLoopError
	 *            Value of the allowable closed-loop error in sensor units (or sensor units per 100ms for velocity).
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigAllowableClosedloopError(int slotIdx,
			int allowableCloseLoopError, int timeoutMs = 0) = 0;
	/**
	 * Sets the maximum integral accumulator in the given parameter slot.
	 *
	 * @param slotIdx
	 *            Parameter slot for the constant.
	 * @param iaccum
	 *            Value of the maximum integral accumulator (closed loop error
	 *            units X 1ms).
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigMaxIntegralAccumulator(int slotIdx, double iaccum,
			int timeoutMs = 0) = 0;
	/**
	 * Sets the peak closed-loop output.  This peak output is slot-specific and
	 *   is applied to the output of the associated PID loop.
	 * This setting is seperate from the generic Peak Output setting.
	 *
	 * @param slotIdx
	 *            Parameter slot for the constant.
	 * @param percentOut
	 *            Peak Percent Output from 0 to 1.  This value is absolute and
	 *						the magnitude will apply in both forward and reverse directions.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigClosedLoopPeakOutput(int slotIdx, double percentOut, int timeoutMs = 0) = 0;
	/**
	 * Sets the loop time (in milliseconds) of the PID closed-loop calculations.
	 * Default value is 1 ms.
	 *
	 * @param slotIdx
	 *            Parameter slot for the constant.
	 * @param loopTimeMs
	 *            Loop timing of the closed-loop calculations.  Minimum value of
	 *						1 ms, maximum of 64 ms.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigClosedLoopPeriod(int slotIdx, int loopTimeMs, int timeoutMs = 0) = 0;
  
	/**
	 * Configures the Polarity of the Auxiliary PID (PID1).
	 *
	 * Standard Polarity:
	 *    Primary Output = PID0 + PID1,
	 *    Auxiliary Output = PID0 - PID1,
	 *
	 * Inverted Polarity:
	 *    Primary Output = PID0 - PID1,
	 *    Auxiliary Output = PID0 + PID1,
	 *
	 * @param invert
	 *            If true, use inverted PID1 output polarity.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code
	 */
	virtual ErrorCode ConfigAuxPIDPolarity(bool invert, int timeoutMs = 0) = 0;

	//------ Close loop State ----------//
	/**
	 * Sets the integral accumulator. Typically this is used to clear/zero the
	 * integral accumulator, however some use cases may require seeding the
	 * accumulator for a faster response.
	 *
	 * @param iaccum
	 *            Value to set for the integral accumulator (closed loop error
	 *            units X 1ms).
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode SetIntegralAccumulator(double iaccum, int pidIdx = 0,
			int timeoutMs = 0) = 0;
	/**
	 * Gets the closed-loop error. The units depend on which control mode is in
	 * use.
	 *
	 * If closed-loop is seeking a target sensor position, closed-loop error is the difference between target
	 * and current sensor value (in sensor units.  Example 4096 units per rotation for CTRE Mag Encoder).
	 *
	 * If closed-loop is seeking a target sensor velocity, closed-loop error is the difference between target
	 * and current sensor value (in sensor units per 100ms).
	 *
	 * If using motion profiling or Motion Magic, closed loop error is calculated against the current target,
	 * and not the "final" target at the end of the profile/movement.
	 *
	 * See Phoenix-Documentation information on units.
	 *
	 *
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @return Closed-loop error value.
	 */
	virtual int GetClosedLoopError(int pidIdx = 0) = 0;
	/**
	 * Gets the iaccum value.
	 *
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @return Integral accumulator value (Closed-loop error X 1ms).
	 */
	virtual double GetIntegralAccumulator(int pidIdx = 0) = 0;
	/**
	 * Gets the derivative of the closed-loop error.
	 *
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @return The error derivative value.
	 */
	virtual double GetErrorDerivative(int pidIdx = 0) = 0;

	/**
	 * Selects which profile slot to use for closed-loop control.
	 *
	 * @param slotIdx
	 *            Profile slot to select.
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 **/
	virtual ErrorCode SelectProfileSlot(int slotIdx, int pidIdx) = 0;

	/**
	 * Gets the current target of a given closed loop.
	 *
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @return The closed loop target.
	 */
	virtual double GetClosedLoopTarget(int pidIdx = 0) = 0;
	/**
	 * Gets the active trajectory target position for using
	 * MotionMagic/MotionProfile control modes.
	 *
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @return The Active Trajectory Position in sensor units.
	 */
	virtual int GetActiveTrajectoryPosition(int pidIdx = 0) = 0;
	/**
	 * Gets the active trajectory target velocity for using
	 * MotionMagic/MotionProfile control modes.
	 *
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @return The Active Trajectory Velocity in sensor units per 100ms.
	 */
	virtual int GetActiveTrajectoryVelocity(int pidIdx = 0) = 0;
	/**
	 * Gets the active trajectory arbitrary feedforward using
	 * MotionMagic/MotionProfile control modes.
	 *
	 * @param pidIdx
	 *            0 for Primary closed-loop. 1 for auxiliary closed-loop.
	 * @return The Active Trajectory ArbFeedFwd in units of percent output
	 * 			(where 0.01 is 1%).
	 */
	virtual double GetActiveTrajectoryArbFeedFwd(int pidIdx = 0) = 0;
	/**
	 * Gets the active trajectory target heading using
	 * MotionMagicArc/MotionProfileArc control modes.
	 *
	 * @return The Active Trajectory Heading in degrees.
	 * @deprecated Use {@link #GetActiveTrajectoryPosition(int)} with 1 passed as an argument instead.
	 */
	virtual double GetActiveTrajectoryHeading() = 0;

	//------ Motion Profile Settings used in Motion Magic  ----------//
	/**
	 * Sets the Motion Magic Cruise Velocity. This is the peak target velocity
	 * that the motion magic curve generator can use.
	 *
	 * @param sensorUnitsPer100ms
	 *            Motion Magic Cruise Velocity (in raw sensor units per 100 ms).
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigMotionCruiseVelocity(int sensorUnitsPer100ms,
			int timeoutMs = 0) = 0;
	/**
	 * Sets the Motion Magic Acceleration. This is the target acceleration that
	 * the motion magic curve generator can use.
	 *
	 * @param sensorUnitsPer100msPerSec
	 *            Motion Magic Acceleration (in raw sensor units per 100 ms per
	 *            second).
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigMotionAcceleration(int sensorUnitsPer100msPerSec,
			int timeoutMs = 0) = 0;

	/**
	 * Sets the Motion Magic S Curve Strength.
 	 * Call this before using Motion Magic.
	 * Modifying this during a Motion Magic action should be avoided.
 	 *
	 * @param curveStrength
 	 *            0 to use Trapezoidal Motion Profile. [1,8] for S-Curve (greater value yields greater smoothing).
	 * @param timeoutMs
 	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
 	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigMotionSCurveStrength(int curveStrength, int timeoutMs) = 0;
	//------ Motion Profile Buffer ----------//
	/**
	 * Clear the buffered motion profile in both controller's RAM (bottom), and in the
	 * API (top).
	 * 
	 * @return Error Code generated by function. 0 indicates no error
	 */
	virtual ErrorCode ClearMotionProfileTrajectories()= 0;
	/**
	 * Retrieve just the buffer count for the api-level (top) buffer. This
	 * routine performs no CAN or data structure lookups, so its fast and ideal
	 * if caller needs to quickly poll the progress of trajectory points being
	 * emptied into controller's RAM. Otherwise just use GetMotionProfileStatus.
	 *
	 * @return number of trajectory points in the top buffer.
	 */
	virtual int GetMotionProfileTopLevelBufferCount()= 0;
	/**
	 * Push another trajectory point into the top level buffer (which is emptied
	 * into the motor controller's bottom buffer as room allows).
	 * @param trajPt to push into buffer.
	 * The members should be filled in with these values...
	 *
	 * 		targPos:  servo position in sensor units.
	 *		targVel:  velocity to feed-forward in sensor units
	 *                 per 100ms.
	 * 		profileSlotSelect0  Which slot to get PIDF gains. PID is used for position servo. F is used
	 *						   as the Kv constant for velocity feed-forward. Typically this is hardcoded
	 *						   to the a particular slot, but you are free gain schedule if need be.
	 *						   Choose from [0,3]
	 *		profileSlotSelect1 Which slot to get PIDF gains for auxiliary PId.
	 *						   This only has impact during MotionProfileArc Control mode.
	 *						   Choose from [0,1].
	 * 	   isLastPoint  set to nonzero to signal motor controller to keep processing this
	 *                     trajectory point, instead of jumping to the next one
	 *                     when timeDurMs expires.  Otherwise MP executer will
	 *                     eventually see an empty buffer after the last point
	 *                     expires, causing it to assert the IsUnderRun flag.
	 *                     However this may be desired if calling application
	 *                     never wants to terminate the MP.
	 *		zeroPos  set to nonzero to signal motor controller to "zero" the selected
	 *                 position sensor before executing this trajectory point.
	 *                 Typically the first point should have this set only thus
	 *                 allowing the remainder of the MP positions to be relative to
	 *                 zero.
	 *		timeDur Duration to apply this trajectory pt.
	 * 				This time unit is ADDED to the exising base time set by
	 * 				configMotionProfileTrajectoryPeriod().
	 * @return CTR_OKAY if trajectory point push ok. ErrorCode if buffer is
	 *         full due to kMotionProfileTopBufferCapacity.
	 */
	virtual ErrorCode PushMotionProfileTrajectory(
			const ctre::phoenix::motion::TrajectoryPoint & trajPt)= 0;
	/**
	 * Retrieve just the buffer full for the api-level (top) buffer. This
	 * routine performs no CAN or data structure lookups, so its fast and ideal
	 * if caller needs to quickly poll. Otherwise just use
	 * GetMotionProfileStatus.
	 *
	 * @return number of trajectory points in the top buffer.
	 */
	virtual bool IsMotionProfileTopLevelBufferFull()= 0;
	/**
	 * This must be called periodically to funnel the trajectory points from the
	 * API's top level buffer to the controller's bottom level buffer. Recommendation
	 * is to call this twice as fast as the execution rate of the motion
	 * profile. So if MP is running with 20ms trajectory points, try calling
	 * this routine every 10ms. All motion profile functions are thread-safe
	 * through the use of a mutex, so there is no harm in having the caller
	 * utilize threading.
	 */
	virtual void ProcessMotionProfileBuffer()= 0;
	/**
	 * Retrieve all status information.
	 * For best performance, Caller can snapshot all status information regarding the
	 * motion profile executer.
	 *
	 * @param statusToFill  Caller supplied object to fill.
	 *
	 * The members are filled, as follows...
	 *
	 *	topBufferRem:	The available empty slots in the trajectory buffer.
	 * 	 				The robot API holds a "top buffer" of trajectory points, so your applicaion
	 * 	 				can dump several points at once.  The API will then stream them into the
	 * 	 		 		low-level buffer, allowing the motor controller to act on them.
	 *
	 *	topBufferRem: The number of points in the top trajectory buffer.
	 *
	 *	btmBufferCnt: The number of points in the low level controller buffer.
	 *
	 *	hasUnderrun: 	Set if isUnderrun ever gets set.
	 * 	 	 	 	 	Can be manually cleared by clearMotionProfileHasUnderrun() or automatically cleared by startMotionProfile().
	 *
	 *	isUnderrun:		This is set if controller needs to shift a point from its buffer into
	 *					the active trajectory point however
	 *					the buffer is empty.
	 *					This gets cleared automatically when is resolved.
	 *
	 *	activePointValid:	True if the active trajectory point is not empty, false otherwise. The members in activePoint are only valid if this signal is set.
	 *
	 *	isLast:	is set/cleared based on the MP executer's current
	 *                trajectory point's IsLast value.  This assumes
	 *                IsLast was set when PushMotionProfileTrajectory
	 *                was used to insert the currently processed trajectory
	 *                point.
	 *
	 *	profileSlotSelect: The currently processed trajectory point's
	 *      			  selected slot.  This can differ in the currently selected slot used
	 *       				 for Position and Velocity servo modes
	 *
	 *	outputEnable:		The current output mode of the motion profile
	 *						executer (disabled, enabled, or hold).  When changing the set()
	 *						value in MP mode, it's important to check this signal to
	 *						confirm the change takes effect before interacting with the top buffer.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode GetMotionProfileStatus(
			ctre::phoenix::motion::MotionProfileStatus & statusToFill)= 0;
	/**
	 * Clear the "Has Underrun" flag. Typically this is called after application
	 * has confirmed an underrun had occured.
	 *
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ClearMotionProfileHasUnderrun(int timeoutMs = 0)= 0;
	/**
	 * Calling application can opt to speed up the handshaking between the robot
	 * API and the controller to increase the download rate of the controller's Motion
	 * Profile. Ideally the period should be no more than half the period of a
	 * trajectory point.
	 *
	 * @param periodMs
	 *            The transmit period in ms.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ChangeMotionControlFramePeriod(int periodMs)= 0;
	/**
	 * When trajectory points are processed in the motion profile executer, the MPE determines
	 * how long to apply the active trajectory point by summing baseTrajDurationMs with the
	 * timeDur of the trajectory point (see TrajectoryPoint).
	 *
	 * This allows general selection of the execution rate of the points with 1ms resolution,
	 * while allowing some degree of change from point to point.
	 * @param baseTrajDurationMs The base duration time of every trajectory point.
	 * 							This is summed with the trajectory points unique timeDur.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigMotionProfileTrajectoryPeriod(int baseTrajDurationMs, int timeoutMs = 0)=0;
    
    //------Feedback Device Interaction Settings---------//
    /**
	 * Disables continuous tracking of the position for analog and pulse-width.
	 * If the signal goes from 4095 to 0 (pulse-width) a motor controller will continue to read 4096 by default.
	 * If overflow tracking is disabled, it will wrap to 0 (not continuous)
	 *
	 * If using pulse-width on CTRE Mag Encoder (within one rotation) or absolute analog sensor (within one rotation),
	 * setting feedbackNotContinuous to true is recommended, to prevent intermittent
	 * connections from causing sensor "jumps" of 4096 (or 1024 for analog) units.
	 *
	 * @param feedbackNotContinuous     True to disable the overflow tracking.
     *
     * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
    virtual ErrorCode ConfigFeedbackNotContinuous(bool feedbackNotContinuous, int timeoutMs = 0) = 0;
    /**
     * Disables going to neutral (brake/coast) when a remote sensor is no longer detected.
     *
     * @param remoteSensorClosedLoopDisableNeutralOnLOS     disable going to neutral 
     *
     * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
    virtual ErrorCode ConfigRemoteSensorClosedLoopDisableNeutralOnLOS(bool remoteSensorClosedLoopDisableNeutralOnLOS, int timeoutMs = 0) = 0;
    /**
     * Enables clearing the position of the feedback sensor when the forward 
     * limit switch is triggered
     *
     * @param clearPositionOnLimitF     Whether clearing is enabled, defaults false
     * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
    virtual ErrorCode ConfigClearPositionOnLimitF(bool clearPositionOnLimitF, int timeoutMs = 0) = 0;
    /**
     * Enables clearing the position of the feedback sensor when the reverse 
     * limit switch is triggered
     *
     * @param clearPositionOnLimitR     Whether clearing is enabled, defaults false
     * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
    virtual ErrorCode ConfigClearPositionOnLimitR(bool clearPositionOnLimitR, int timeoutMs = 0) = 0;
    /**
     * Enables clearing the position of the feedback sensor when the quadrature index signal
     * is detected
     *
     * @param clearPositionOnQuadIdx    Whether clearing is enabled, defaults false
     * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
    virtual ErrorCode ConfigClearPositionOnQuadIdx(bool clearPositionOnQuadIdx, int timeoutMs = 0) = 0;
    /**
     * Disables limit switches triggering (if enabled) when the sensor is no longer detected.
     *
     * @param limitSwitchDisableNeutralOnLOS    disable triggering
     *
     * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
    virtual ErrorCode ConfigLimitSwitchDisableNeutralOnLOS(bool limitSwitchDisableNeutralOnLOS, int timeoutMs = 0) = 0;
    /**
     * Disables soft limits triggering (if enabled) when the sensor is no longer detected.
     *
     * @param softLimitDisableNeutralOnLOS    disable triggering
     *
     * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
    virtual ErrorCode ConfigSoftLimitDisableNeutralOnLOS(bool softLimitDisableNeutralOnLOS, int timeoutMs = 0) = 0;
    /**
     * Sets the edges per rotation of a pulse width sensor. (This should be set for 
     * tachometer use).
     *
     * @param pulseWidthPeriod_EdgesPerRot    edges per rotation
     *
     * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
    virtual ErrorCode ConfigPulseWidthPeriod_EdgesPerRot(int pulseWidthPeriod_EdgesPerRot, int timeoutMs = 0) = 0;
    /**
     * Sets the number of samples to use in smoothing a pulse width sensor with a rolling 
     * average. Default is 1 (no smoothing).
     *
     * @param pulseWidthPeriod_FilterWindowSz   samples for rolling avg
     *
     * @param timeoutMs
     *            Timeout value in ms. If nonzero, function will wait for
     *            config success and report an error if it times out.
     *            If zero, no blocking or checking is performed.
     * @return Error Code generated by function. 0 indicates no error.
     */
    virtual ErrorCode ConfigPulseWidthPeriod_FilterWindowSz(int pulseWidthPeriod_FilterWindowSz, int timeoutMs = 0) = 0;
	
    //------ error ----------//
	/**
	 * Gets the last error generated by this object. Not all functions return an
	 * error code but can potentially report errors. This function can be used
	 * to retrieve those error codes.
	 *
	 * @return Last Error Code generated by a function.
	 */
	virtual ErrorCode GetLastError() = 0;

	//------ Faults ----------//
	/**
	 * Polls the various fault flags.
	 *
	 * @param toFill
	 *            Caller's object to fill with latest fault flags.
	 * @return Last Error Code generated by a function.
	 */
	virtual ErrorCode GetFaults(Faults & toFill) = 0;
	/**
	 * Polls the various sticky fault flags.
	 *
	 * @param toFill
	 *            Caller's object to fill with latest sticky fault flags.
	 * @return Last Error Code generated by a function.
	 */
	virtual ErrorCode GetStickyFaults(StickyFaults & toFill) = 0;
	/**
	 * Clears all sticky faults.
	 *
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Last Error Code generated by a function.
	 */
	virtual ErrorCode ClearStickyFaults(int timeoutMs = 0) = 0;

	//------ Firmware ----------//
	/**
	 * Gets the firmware version of the device.
	 *
	 * @return Firmware version of device. For example: version 1-dot-2 is
	 *         0x0102.
	 */
	virtual int GetFirmwareVersion() = 0;
	/**
	 * Returns true if the device has reset since last call.
	 *
	 * @return Has a Device Reset Occurred?
	 */
	virtual bool HasResetOccurred() = 0;

	//------ Custom Persistent Params ----------//
	/**
	 * Sets the value of a custom parameter. This is for arbitrary use.
	 *
	 * Sometimes it is necessary to save calibration/limit/target information in
	 * the device. Particularly if the device is part of a subsystem that can be
	 * replaced.
	 *
	 * @param newValue
	 *            Value for custom parameter.
	 * @param paramIndex
	 *            Index of custom parameter [0,1]
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigSetCustomParam(int newValue, int paramIndex,
			int timeoutMs = 0) = 0;
	/**
	 * Gets the value of a custom parameter.
	 *
	 * @param paramIndex
	 *            Index of custom parameter [0,1].
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Value of the custom param.
	 */
	virtual int ConfigGetCustomParam(int paramIndex, int timeoutMs = 0) = 0;

	//------ Generic Param API, typically not used ----------//
	/**
	 * Sets a parameter. Generally this is not used. This can be utilized in -
	 * Using new features without updating API installation. - Errata
	 * workarounds to circumvent API implementation. - Allows for rapid testing
	 * / unit testing of firmware.
	 *
	 * @param param
	 *            Parameter enumeration.
	 * @param value
	 *            Value of parameter.
	 * @param subValue
	 *            Subvalue for parameter. Maximum value of 255.
	 * @param ordinal
	 *            Ordinal of parameter.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for config
	 *            success and report an error if it times out. If zero, no
	 *            blocking or checking is performed.
	 * @return Error Code generated by function. 0 indicates no error.
	 */
	virtual ErrorCode ConfigSetParameter(ParamEnum param, double value,
			uint8_t subValue, int ordinal, int timeoutMs = 0) = 0;
	/**
	 * Gets a parameter.
	 *
	 * @param paramEnum
	 *            Parameter enumeration.
	 * @param ordinal
	 *            Ordinal of parameter.
	 * @param timeoutMs
	 *            Timeout value in ms. If nonzero, function will wait for
	 *            config success and report an error if it times out.
	 *            If zero, no blocking or checking is performed.
	 * @return Value of parameter.
	 */
	virtual double ConfigGetParameter(ParamEnum paramEnum, int ordinal,
			int timeoutMs = 0) = 0;

	//------ Misc. ----------//
	/**
	 * @return BaseID of device
	 */
	virtual int GetBaseID() = 0;
	/**
	 * Returns the Device ID
	 *
	 * @return Device number.
	 */
	virtual int GetDeviceID() = 0;
	/**
	 * @return control mode motor controller is in
	 */
	virtual ControlMode GetControlMode() = 0;

	// ----- Follower ------//
	/* in parent interface */
};

}
} // namespace phoenix
}
