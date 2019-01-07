#pragma once

#include "ctre/phoenix/motorcontrol/ControlMode.h"
#include "ctre/phoenix/motorcontrol/ControlFrame.h"
#include "ctre/phoenix/motorcontrol/NeutralMode.h"
#include "ctre/phoenix/motorcontrol/FeedbackDevice.h"
#include "ctre/phoenix/motorcontrol/SensorCollection.h"
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

class IMotorControllerEnhanced: public virtual IMotorController {
public:
	virtual ~IMotorControllerEnhanced() {
	}

	//------ Set output routines. ----------//
	/* in parent */

	//------ Invert behavior ----------//
	/* in parent */

	//----- Factory Default Configuration -----//
	/* in parent */

	//----- general output shaping ------------------//
	/* in parent */

	//------ Voltage Compensation ----------//
	/* in parent */

	//------ General Status ----------//
	/* in parent */

	//------ sensor selection ----------//
	/* expand the options */
	virtual ErrorCode ConfigSelectedFeedbackSensor(
			FeedbackDevice feedbackDevice, int pidIdx = 0, int timeoutMs = 0) = 0;
	virtual ErrorCode ConfigSelectedFeedbackSensor(
			RemoteFeedbackDevice feedbackDevice, int pidIdx = 0, int timeoutMs = 0) = 0;

	//------- sensor status --------- //
	/* in parent */

	//------ status frame period changes ----------//
	virtual ErrorCode SetStatusFramePeriod(StatusFrame frame, uint8_t periodMs,
			int timeoutMs = 0) = 0;
	virtual ErrorCode SetStatusFramePeriod(StatusFrameEnhanced frame,
			uint8_t periodMs, int timeoutMs = 0) = 0;
	virtual int GetStatusFramePeriod(StatusFrame frame, int timeoutMs = 0) = 0;
	virtual int GetStatusFramePeriod(StatusFrameEnhanced frame, int timeoutMs = 0) = 0;

	//------ General Status ----------//
	virtual double GetOutputCurrent() = 0;

	//----- velocity signal conditionaing ------//
	virtual ErrorCode ConfigVelocityMeasurementPeriod(VelocityMeasPeriod period,
			int timeoutMs = 0)= 0;
	virtual ErrorCode ConfigVelocityMeasurementWindow(int windowSize,
			int timeoutMs = 0)= 0;

	//------ remote limit switch ----------//
	virtual ErrorCode ConfigForwardLimitSwitchSource(
			RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
			int deviceID, int timeoutMs = 0) = 0;
	virtual ErrorCode ConfigReverseLimitSwitchSource(
			RemoteLimitSwitchSource type, LimitSwitchNormal normalOpenOrClose,
			int deviceID, int timeoutMs = 0) = 0;

	//------ local limit switch ----------//
	virtual ErrorCode ConfigForwardLimitSwitchSource(LimitSwitchSource type,
			LimitSwitchNormal normalOpenOrClose, int timeoutMs = 0)= 0;
	virtual ErrorCode ConfigReverseLimitSwitchSource(LimitSwitchSource type,
			LimitSwitchNormal normalOpenOrClose, int timeoutMs = 0)= 0;

	//------ soft limit ----------//
	/* in parent */

	//------ RAW Sensor API ----------//
	/**
	 * @return object that can get/set individual RAW sensor values.
	 */
	ctre::phoenix::motorcontrol::SensorCollection & GetSensorCollection();

	//------ Current Lim ----------//
	virtual ErrorCode ConfigPeakCurrentLimit(int amps, int timeoutMs = 0)= 0;
	virtual ErrorCode ConfigPeakCurrentDuration(int milliseconds,
			int timeoutMs = 0)= 0;
	virtual ErrorCode ConfigContinuousCurrentLimit(int amps, int timeoutMs = 0)= 0;
	virtual void EnableCurrentLimit(bool enable)= 0;

	//------ General Close loop ----------//
	/* in parent */

	//------ Motion Profile Settings used in Motion Magic and Motion Profile ----------//
	/* in parent */

	//------ Motion Profile Buffer ----------//
	/* in parent */

	//------ error ----------//
	/* in parent */

	//------ Faults ----------//
	/* in parent */

	//------ Firmware ----------//
	/* in parent */

	//------ Custom Persistent Params ----------//
	/* in parent */

	//------ Generic Param API, typically not used ----------//
	/* in parent */

	//------ Misc. ----------//
	/* in parent */

}; // class IMotorControllerEnhanced
} // namespace motorcontrol
} // namespace phoenix
} // namespace ctre
