#pragma once

#include <vector>
#include "ctre/phoenix/CANifier.h"
#include "ctre/phoenix/tasking/IProcessable.h"

namespace ctre{
namespace phoenix {
	
/**
 * Class to convert RC PWM signals into CAN signals using CANifier
 */
class RCRadio3Ch : public ctre::phoenix::tasking::IProcessable{
public:
	/**
	 * CANifier PWM Channel
	 */
	enum Channel{
		/**
		 * PWM Channel 1
		 */
		Channel1,
		/**
		 * PWM Channel 2
		 */
		Channel2,
		/**
		 * PWM Channel 3
		 */
		Channel3,
	};
	/**
	 * Status of CANiifer
	 */
	enum Status{
		/**
		 * CANifier is not communicating over CAN
		 */
		LossOfCAN,
		/**
		 * CANifier does not detect PWM input
		 */
		LossOfPwm,
		/**
		 * CANifier is OK
		 */
		Okay,
	};
	
	/**
	 * Current status of CANifier
	 */
	Status CurrentStatus = Status::Okay;

	/**
	 * Constructor for RCRadio3Ch
	 * @param canifier reference to CANifier to use
	 */
	RCRadio3Ch(ctre::phoenix::CANifier *canifier);
	/**
	 * Gets the microsecond duty cycle of specified channel
	 * @param channel channel to read
	 * @return duty cycle in micro seconds
	 */
	double GetDutyCycleUs(Channel channel);
	/**
	 * Gets the percent duty cycle of specified channel
	 * @param channel channel to read
	 * @return duty cycle in percent [0,1]
	 */
	double GetDutyCyclePerc(Channel channel);
	/**
	 * Gets whether a switch is on or off for a channel
	 * @param channel channel to read
	 * @return true if switch is on
	 */
	bool GetSwitchValue(Channel channel);
	/**
	 * Gets the period of the pwm signal of a channel
	 * @param channel channel to read
	 * @return period of pwm signal in microseconds
	 */
	double GetPeriodUs(Channel channel);

	//ILoopable
	/**
	 * Call this once every loop, it processes the incoming CANifier signals
	 */
	void Process();

private:
	int _errorCodes[4];
	ctre::phoenix::CANifier *_canifier;


	//This is only a 2d array??
	double _pulseWidthAndPeriods[4][2] =
	{
			{ 0, 0 },
			{ 0, 0 },
			{ 0, 0 },
			{ 0, 0 },
	};
	double Interpolate(std::vector<double> &xData, std::vector<double> &yData, double x, bool extrapolate);
};

}}
