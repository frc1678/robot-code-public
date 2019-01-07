#pragma once

#include <vector>
#include "ctre/phoenix/CANifier.h"
#include "ctre/phoenix/tasking/IProcessable.h"

namespace ctre{
namespace phoenix {
	
class RCRadio3Ch : public ctre::phoenix::tasking::IProcessable{
public:
	enum Channel{
		Channel1,
		Channel2,
		Channel3,
	};
	enum Status{
		LossOfCAN,
		LossOfPwm,
		Okay,
	};
	Status CurrentStatus = Status::Okay;

	RCRadio3Ch(ctre::phoenix::CANifier *canifier);
	double GetDutyCycleUs(Channel channel);
	double GetDutyCyclePerc(Channel channel);
	bool GetSwitchValue(Channel channel);
	double GetPeriodUs(Channel channel);

	//ILoopable
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
