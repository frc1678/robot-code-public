#include "muan/wpilib/ds_sender.h"

#include "WPILib.h"

namespace muan {

namespace wpilib {

DriverStationSender::DriverStationSender(DriverStationQueue* ds_queue, GameSpecificStringQueue* gss_queue)
    : ds_queue_{ds_queue}, gss_queue_{gss_queue} {}

void DriverStationSender::Send() {
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
  status->set_brownout(RobotController::IsBrownedOut());
  status->set_has_ds_connection(DriverStation::GetInstance().IsDSAttached());

  status->set_match_time(DriverStation::GetInstance().GetMatchTime());
  status->set_alliance(
      static_cast<DriverStationStatus::Alliance>(DriverStation::GetInstance().GetAlliance()));
  status->set_driver_station_position(DriverStation::GetInstance().GetLocation());
  status->set_has_fms_connection(DriverStation::GetInstance().IsFMSAttached());
  status->set_is_sys_active(RobotController::IsSysActive());

  ds_queue_->WriteMessage(status);

  std::string game_string = DriverStation::GetInstance().GetGameSpecificMessage();
  if (!game_string.empty() && gss_queue_) {
    GameSpecificStringProto gss;
    gss->set_code(game_string);
    gss_queue_->WriteMessage(gss);
  }
}

}  // namespace wpilib

}  // namespace muan
