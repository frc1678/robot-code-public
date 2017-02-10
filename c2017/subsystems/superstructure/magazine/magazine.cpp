#include "c2017/subsystems/superstructure/magazine/magazine.h"

namespace c2017 {

namespace magazine {

MagazineOutputProto Magazine::Update(MagazineInputProto input, const DriverStationStatus& robot_state) {
  has_hp_gear_ = input->has_hp_gear();
  double upper_voltage = 0;
  double side_voltage = 0;
  bool gear_intake_covered = true;

  double gear_shutter_open = score_gear_;

  bool enable_outputs = !(robot_state.mode() == RobotMode::DISABLED || robot_state.mode() == RobotMode::ESTOP || robot_state.brownout());

  if (enable_outputs) {
    switch (hp_intake_goal_) {
      case NONE:
        gear_intake_covered = true;
        break;
      case BALLS:
        gear_intake_covered = true;
        break;
      case GEAR:
        gear_intake_covered = false;
        break;
      case BOTH:
        if (input->has_hp_gear()) {
          gear_intake_covered = true;
        } else {
          gear_intake_covered = false;
        }
        break;
    }

    switch (side_goal_) {
      case SIDE_IDLE:
        side_voltage = 0;
        break;
      case SIDE_PULL_IN:
        side_voltage = 12;
        break;
      case SIDE_AGITATE:
        side_voltage = -12;
        break;
    }

    switch (upper_goal_) {
      case UPPER_IDLE:
        upper_voltage = 0;
        break;
      case UPPER_FORWARD:
        upper_voltage = 12;
        break;
      case UPPER_BACKWARD:
        upper_voltage = -12;
        break;
    }
  } else {
    gear_intake_covered = false;
    magazine_extended_ = false; // true?
    gear_shutter_open = false;
    upper_voltage = 0;
    side_voltage = 0;
  }

  c2017::magazine::MagazineOutputProto output;

  output->set_gear_intake_covered(gear_intake_covered);
  output->set_magazine_extended(magazine_extended_);
  output->set_gear_shutter_open(gear_shutter_open);
  output->set_upper_voltage(upper_voltage);
  output->set_side_voltage(side_voltage);

  return output;
}

void Magazine::SetGoal(MagazineGoalProto goal) {
  hp_intake_goal_ = goal->hp_intake_goal();
  upper_goal_ = goal->upper_goal();
  magazine_extended_ = goal->magazine_extended();
  score_gear_ = goal->score_gear();
  side_goal_ = goal->side_goal();
}

}  // namespace magazine

}  // namespace c2017
