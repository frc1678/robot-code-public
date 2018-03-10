#include <string>
#include "WPILib.h"
#include "muan/wpilib/ds_sender.h"

namespace muan {

namespace wpilib {

DriverStationSender::DriverStationSender(DriverStationQueue* ds_queue,
                                         GameSpecificStringQueue* gss_queue)
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
      static_cast<Alliance>(DriverStation::GetInstance().GetAlliance()));
  status->set_driver_station_position(
      DriverStation::GetInstance().GetLocation());
  status->set_has_fms_connection(DriverStation::GetInstance().IsFMSAttached());
  status->set_is_sys_active(RobotController::IsSysActive());

  HAL_MatchInfo match_info;
  auto hal_call_status = HAL_GetMatchInfo(&match_info);
  if (hal_call_status == 0) {
    status->set_match_type(
        static_cast<DriverStationStatus::MatchType>(match_info.matchType));
    status->set_match_number(match_info.matchNumber);

    if (match_info.gameSpecificMessage &&
        std::strlen(match_info.gameSpecificMessage) > 0 && gss_queue_) {
      GameSpecificStringProto gss;
      gss->set_code(match_info.gameSpecificMessage);
      gss_queue_->WriteMessage(gss);
    }
  }
  HAL_FreeMatchInfo(&match_info);

  ds_queue_->WriteMessage(status);
}

}  // namespace wpilib

}  // namespace muan
