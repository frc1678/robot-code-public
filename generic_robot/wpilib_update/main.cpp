#include "generic_robot/wpilib_update/main.h"
#include "WPILib.h"
#include "generic_robot/queue_manager/queue_manager.h"
#include "muan/wpilib/queue_types.h"

namespace generic_robot {

namespace wpilib_update {

Main::Main()
    : throttle_{1}, wheel_{0}, gamepad_{2}, ds_sender_{&QueueManager::GetInstance().driver_station_queue()} {
  shifting_low_ = throttle_.MakeButton(4);
  shifting_high_ = throttle_.MakeButton(5);
  quickturn_ = throttle_.MakeButton(5);
}

void Main::Update() {
  if (DriverStation::GetInstance().IsAutonomous()) {
  } else if (DriverStation::GetInstance().IsOperatorControl()) {
    // Update joysticks
    throttle_.Update();
    wheel_.Update();
    gamepad_.Update();
  }
  ds_sender_.Send();
}

}  // namespace wpilib_update

}  // namespace generic_robot
