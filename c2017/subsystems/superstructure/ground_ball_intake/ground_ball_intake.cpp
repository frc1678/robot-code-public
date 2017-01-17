#include "c2017/subsystems/superstructure/ground_ball_intake/ground_ball_intake.h"

namespace c2017 {

namespace ball_intake {

GroundBallIntake::GroundBallIntake() : output_(), status_(), goal_(){


}

GroundBallIntakeOutputProto GroundBallIntake::Update(const DriverStationStatus& robot_state) {
  double roller_voltage = 0;

  bool enable_outputs = !(robot_state.mode() == RobotMode::DISABLED || robot_state.mode() == RobotMode::ESTOP);

  if (enable_outputs) {
    if (goal_->run_intake() == RollerGoal::INTAKE) {
      roller_voltage = 12;
    } else if (goal_->run_intake() == RollerGoal::OUTTAKE) {
      roller_voltage = -12;
    } else if (goal_->run_intake() == RollerGoal::IDLE) {
      roller_voltage = 0;
    }
  } else {
    roller_voltage = 0;
  }

  output_->set_roller_voltage(roller_voltage);
  output_->set_intake_up(goal_->intake_up());

  status_->set_running(goal_->run_intake());
  status_->set_is_intake_up(goal_->intake_up());

  return output_;
  }

void GroundBallIntake::SetGoal(const GroundBallIntakeGoalProto& goal) {
  goal_ = goal;
}

GroundBallIntakeStatusProto GroundBallIntake::get_status() {
  return status_;
}

} //ball_intake

} //c2017
