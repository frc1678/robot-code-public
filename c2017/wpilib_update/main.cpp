#include "c2017/wpilib_update/main.h"
#include "WPILib.h"
#include "muan/wpilib/queue_types.h"
#include "c2017/queue_manager/queue_manager.h"

namespace c2017 {
namespace citrus_robot {

CitrusRobot::CitrusRobot() : throttle_{1}, wheel_{0}, gamepad_{2} {
  shifting_low_ = throttle_.MakeButton(4);
  shifting_high_ = throttle_.MakeButton(5);
  quickturn_ = throttle_.MakeButton(5);
}

void CitrusRobot::Update() {
  if (DriverStation::GetInstance().IsAutonomous()) {
    lemonscript_.Start();  // Weird to call Start in a loop, but it's a setter so it's fine

  } else if (DriverStation::GetInstance().IsOperatorControl()) {
    lemonscript_.Stop();  // Weirder to do this, but it works :/

    // Update joysticks
    throttle_.Update();
    wheel_.Update();
    gamepad_.Update();
  }

  SendDSMessage();
}

void CitrusRobot::SendDSMessage() {
  muan::wpilib::DriverStationProto status;

  if (DriverStation::GetInstance().IsDisabled()) {
    status->set_mode(RobotMode::DISABLED);
  } else if (DriverStation::GetInstance().IsAutonomous()) {
    status->set_mode(RobotMode::AUTONOMOUS);
  } else if (DriverStation::GetInstance().IsOperatorControl()) {
    status->set_mode(RobotMode::TELEOP);
  } else {
    status->set_mode(RobotMode::ESTOP);
  }

  status->set_battery_voltage(DriverStation::GetInstance().GetBatteryVoltage());
  status->set_brownout(DriverStation::GetInstance().IsBrownedOut());
  status->set_has_ds_connection(DriverStation::GetInstance().IsDSAttached());

  c2017::QueueManager::GetInstance().driver_station_queue()->WriteMessage(status);
}

}  // namespace citrus_robot
}  // namespace c2017
