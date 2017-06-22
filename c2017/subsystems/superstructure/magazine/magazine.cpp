#include "c2017/subsystems/superstructure/magazine/magazine.h"

namespace c2017 {

namespace magazine {

Magazine::Magazine() : lower_ramping_{0.1}, side_ramping_{0.1} {}

MagazineOutputProto Magazine::Update(bool outputs_enabled) {
  double upper_voltage = 0;
  double side_voltage = 0;
  double lower_voltage = 0;

  if (outputs_enabled) {
    switch (side_goal_) {
      case SIDE_IDLE:
        side_voltage = 0;
        magazine_status_->set_side_conveyor_running(false);
        break;
      case SIDE_PULL_IN:
        side_voltage = 6;
        magazine_status_->set_side_conveyor_running(true);
        break;
      case SIDE_AGITATE:
        side_voltage = -6;
        magazine_status_->set_side_conveyor_running(true);
        break;
    }

    switch (upper_goal_) {
      case UPPER_IDLE:
        upper_voltage = 0;
        magazine_status_->set_upper_conveyor_running(false);
        break;
      case UPPER_FORWARD:
        upper_voltage = 12;
        magazine_status_->set_upper_conveyor_running(true);
        break;
      case UPPER_BACKWARD:
        upper_voltage = -4;
        magazine_status_->set_upper_conveyor_running(true);
        break;
    }

    switch (lower_goal_) {
      case LOWER_IDLE:
        lower_voltage = 0;
        magazine_status_->set_lower_conveyor_running(false);
        break;
      case LOWER_FORWARD:
        lower_voltage = 12;
        magazine_status_->set_lower_conveyor_running(true);
        break;
      case LOWER_BACKWARD:
        lower_voltage = -12;
        magazine_status_->set_lower_conveyor_running(true);
        break;
    }
  } else {
    side_magazine_extended_ = false;
    front_magazine_extended_ = false;
    upper_voltage = 0;
    side_voltage = 0;
  }

  c2017::magazine::MagazineOutputProto output;

  output_->set_side_magazine_extended(side_magazine_extended_);
  output_->set_front_magazine_extended(front_magazine_extended_);
  output_->set_upper_voltage(upper_voltage);
  output_->set_side_voltage(side_ramping_.Update(side_voltage));
  output_->set_lower_voltage(lower_ramping_.Update(lower_voltage));
  QueueManager::GetInstance()->magazine_status_queue()->WriteMessage(magazine_status_);

  return output_;
}

void Magazine::SetGoal(MagazineGoalProto goal) {
  upper_goal_ = goal->upper_goal();
  side_magazine_extended_ = goal->side_magazine_extended();
  front_magazine_extended_ = goal->front_magazine_extended();
  side_goal_ = goal->side_goal();
  lower_goal_ = goal->lower_goal();
}

}  // namespace magazine

}  // namespace c2017
