#include "pid_turret_controller.h"

namespace o2016 {

namespace turret {

PidTurretController::PidTurretController() :
  pid_(20,0,3),
  calibration_(2. * 3.1415 / 37.65) {}

TurretOutputProto PidTurretController::Update(const TurretInputProto& input) {
  current_position_ = calibration_.Update(
      input->encoder_position(), input->pot_position(), input->index_click());
  TurretOutputProto out;
  double e = goal_->goal_angle() - current_position_;
  double out_v = pid_.Calculate(0.005, e);
  if (calibration_.is_calibrated()) {
    out->set_voltage(out_v);
  } else {
    out->set_voltage(1);
  }
  std::cout << out->voltage() << " " << current_position_ << " " << goal_->goal_angle() << "\n";
  return out;
}

void PidTurretController::SetGoal(const TurretGoalProto& goal) {
  double capped_goal = muan::Cap(goal->goal_angle(), -1.4, 1.4);
  goal_->set_goal_angle(capped_goal);
}
}

}
