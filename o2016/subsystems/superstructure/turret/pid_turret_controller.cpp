#include "pid_turret_controller.h"

namespace o2016 {

namespace turret {

PidTurretController::PidTurretController() : pid_(20,0,3) {}

TurretOutputProto PidTurretController::Update(const TurretInputProto& input) {
  current_position_ = input->encoder_position();
  TurretOutputProto out;
  double e = goal_->goal_angle() - current_position_;
  double out_v = pid_.Calculate(0.005, e);
  out->set_voltage(out_v);
  return out;
}

void PidTurretController::SetGoal(const TurretGoalProto& goal) {
  double capped_goal = muan::Cap(goal->goal_angle(), -1.4, 1.4);
  goal_->set_goal_angle(capped_goal);
}
}

}
