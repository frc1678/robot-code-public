#include "c2017/subsystems/superstructure/magazine/magazine.h"

namespace c2017 {

namespace magazine {

MagazineOutputProto Magazine::Update(MagazineInputProto input) {
  has_hp_gear_ = input->has_hp_gear();
  conveyor_current_ = input->conveyor_current();

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
      if (has_hp_gear_) {
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

  if (score_gear_) {
    gear_shutter_open_ = true;
  } else {
    gear_shutter_open_ = false;
  }

  output_->set_gear_intake_covered(gear_intake_covered_);
  output_->set_magazine_extended(magazine_extended_);
  output_->set_gear_shutter_open(gear_shutter_open_);
  output_->set_gear_rotator_voltage(gear_rotator_voltage_);
  output_->set_conveyor_voltage(conveyor_voltage_);
  output_->set_brush_voltage(brush_voltage_);

  return output_;
}

void Magazine::SetGoal(MagazineGoalProto goal) {
  conveyor_goal_ = goal->conveyor_goal();
  hp_intake_goal_ = goal->hp_intake_goal();
  brush_goal_ = goal->brush_goal();
}

}  // magazine

}  // c2017
