#include "c2017/subsystems/superstructure/ground_ball_intake/ground_ball_intake.h"

namespace c2017 {

namespace ground_ball_intake {

GroundBallIntakeOutputProto GroundBallIntake::Update(const DriverStationStatus& robot_state, GroundBallIntakeGoalProto goal_) {
  double roller_voltage = 0;
  bool enable_outputs = !(robot_state.mode() == RobotMode::DISABLED || robot_state.mode() == RobotMode::ESTOP || robot_state.brownout() == true);
  bool intake_up;

  if (enable_outputs) {
    switch(goal_->run_intake()) {
      case RollerGoal::INTAKE :
        roller_voltage = 12;
        break;
      case RollerGoal::OUTTAKE :
        roller_voltage = -12;
        break;
      case RollerGoal::IDLE :
        roller_voltage = 0;
        break;
    }
    intake_up = goal_->intake_up();
  } else {
    roller_voltage = 0;
    intake_up = true;
  }

  output_->set_roller_voltage(roller_voltage);
  output_->set_intake_up(intake_up);

  status_->set_running(goal_->run_intake());
  status_->set_is_intake_up(intake_up);

  return output_;
}

GroundBallIntakeStatusProto GroundBallIntake::get_status() {
  return status_;
}

} //ground_ball_intake

} //c2017
