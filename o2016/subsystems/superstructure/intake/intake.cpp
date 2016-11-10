#include "intake.h"
#include <iostream>

namespace o2016 {

namespace intake {

using namespace ::muan::units;

Intake::Intake()
    : controller_(o2016::intake::IntakeController()), roller_goal_(STOP) {}

IntakeOutputProto Intake::Update(IntakeInputProto input, IntakeGoalProto goal,
                                 bool enabled) {
  roller_goal_ = goal->intake_speed();
  IntakeOutputProto output;

  Voltage arm_voltage;
  Voltage roller_voltage;
  Voltage secondaries_voltage;

  if (roller_goal_ == RollerGoal::FORWARD) {
    roller_voltage = 12.0 * V;
    secondaries_voltage = 4.0 * V;
  } else if (roller_goal_ == RollerGoal::REVERSE) {
    roller_voltage = -12.0 * V;
    secondaries_voltage = -4.0 * V;
  } else if (roller_goal_ == RollerGoal::STOP) {
    roller_voltage = 0.0 * V;
    secondaries_voltage = 0.0 * V;
  }

  arm_voltage =
      controller_.Update(goal->goal_angle(), input->encoder_position(),
                         input->index_click(), enabled);

  output->set_arm_voltage(arm_voltage);
  output->set_roller_voltage(roller_voltage);
  output->set_secondaries_voltage(secondaries_voltage);

  status->set_intake_position(controller_.GetAngle());
  status->set_at_goal(controller_.AtGoal());
  status->set_filtered_angle_goal(controller_.GetAngle());
  status->set_current_roller_goal(roller_goal_);
  status->set_is_calibrated(controller_.is_calibrated());

  return output;
}

IntakeStatusProto Intake::Status() { return status; }
}
}
