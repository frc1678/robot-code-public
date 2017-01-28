#include "c2017/subsystems/superstructure/magazine/magazine.h"

namespace c2017 {

namespace magazine {

MagazineOutputProto Magazine::Update(MagazineInputProto input) {
  has_hp_gear_ = input->has_hp_gear();
  brush_voltage_ = input->brush_voltage();

  double conveyor_voltage_ = 0;
  switch (conveyor_goal_) {
    case CONVEYOR_IDLE:
      conveyor_voltage_ = 0;
      break;

    case CONVEYOR_FORWARD:
      conveyor_voltage_ = 12;
      break;

    case CONVEYOR_BACKWARD:
      conveyor_voltage_ = -12;
      break;
  }

  switch (hp_intake_goal_) {
    case NONE:
      gear_intake_covered_ = true;
      break;

    case BALLS:
      gear_intake_covered_ = true;
      break;

    case GEAR:
      gear_intake_covered_ = false;
      break;

    case BOTH:
      if (input->has_hp_gear()) {
        gear_intake_covered_ = true;
      } else {
        gear_intake_covered_ = false;
      }
      break;
  }

  switch (brush_goal_) {
    case BRUSH_IDLE:
      brush_voltage_ = 0;
      break;

    case BRUSH_FORWARD:
      brush_voltage_ = 12;
      break;

    case BRUSH_BACKWARD:
      brush_voltage_ = -12;
      break;
  }

  double gear_shutter_open = score_gear_;
  double gear_rotator_voltage = rotate_gear_ ? 3 : 0;

  c2017::magazine::MagazineOutputProto output_;

  output_->set_gear_intake_covered(gear_intake_covered_);
  output_->set_magazine_extended(magazine_extended_);
  output_->set_gear_shutter_open(gear_shutter_open);
  output_->set_gear_rotator_voltage(gear_rotator_voltage);
  output_->set_conveyor_voltage(conveyor_voltage_);
  output_->set_brush_voltage(brush_voltage_);
  output_->set_score_gear(score_gear_);

  return output_;
}

void Magazine::SetGoal(MagazineGoalProto goal) {
  conveyor_goal_ = goal->conveyor_goal();
  hp_intake_goal_ = goal->hp_intake_goal();
  brush_goal_ = goal->brush_goal();
  magazine_extended_ = goal->magazine_extended();
  rotate_gear_ = goal->rotate_gear();
  score_gear_ = goal->score_gear();
}

}  // magazine

}  // c2017
