#include "c2017/subsystems/superstructure/magazine/magazine.h"

namespace c2017 {

namespace magazine {

MagazineOutputProto Magazine::Update(MagazineInputProto input, bool outputs_enabled) {
  has_hp_gear_ = input->has_hp_gear();
  double upper_voltage = 0;
  double side_voltage = 0;
  double lower_voltage = 0;
  bool gear_intake_closed = true;

  bool gear_shutter_open = score_gear_;

  if (outputs_enabled) {
    switch (hp_intake_goal_) {
      case NONE:
        gear_intake_closed = true;
        break;
      case BALLS:
        gear_intake_closed = true;
        break;
      case GEAR:
        gear_intake_closed = false;
        break;
      case BOTH:
        if (input->has_hp_gear()) {
          gear_intake_closed = true;
        } else {
          gear_intake_closed = false;
        }
        break;
    }

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
    gear_intake_closed = true;
    magazine_extended_ = false;
    gear_shutter_open = false;
    upper_voltage = 0;
    side_voltage = 0;
  }

  c2017::magazine::MagazineOutputProto output;

  output_->set_gear_intake_closed(gear_intake_closed);
  output_->set_magazine_extended(magazine_extended_);
  output_->set_gear_shutter_open(gear_shutter_open);
  output_->set_upper_voltage(upper_voltage);
  output_->set_side_voltage(side_voltage);
  output_->set_lower_voltage(lower_voltage);
  magazine_status_->set_has_hp_gear(has_hp_gear_);
  QueueManager::GetInstance().magazine_status_queue().WriteMessage(magazine_status_);

  return output_;
}

void Magazine::SetGoal(MagazineGoalProto goal) {
  hp_intake_goal_ = goal->hp_intake_goal();
  upper_goal_ = goal->upper_goal();
  magazine_extended_ = goal->magazine_extended();
  score_gear_ = goal->score_gear();
  side_goal_ = goal->side_goal();
  lower_goal_ = goal->lower_goal();
}

}  // namespace magazine

}  // namespace c2017
