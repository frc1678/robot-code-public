#include "gtest/gtest.h"
#include "muan/wpilib/queue_types.h"
#include "c2017/subsystems/superstructure/ground_ball_intake/queue_types.h"
#include "c2017/subsystems/superstructure/ground_ball_intake/ground_ball_intake.h"

using namespace c2017::ball_intake;

TEST(RollerIntake, IntakeGoingDown) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeStatusProto status;
  GroundBallIntakeGoalProto goal;
  DriverStationStatus ds_status;
  ds_status.set_mode(RobotMode::TELEOP);
  RollerGoal roller_goal = RollerGoal::INTAKE;
  goal->set_intake_up(false);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  intake.SetGoal(goal);
  output = intake.Update(ds_status);
  status = intake.get_status();
  EXPECT_NEAR(output->roller_voltage(), 12, 1e-5);
  EXPECT_FALSE(output->intake_up());
  EXPECT_FALSE(status->is_intake_up());
  EXPECT_EQ(status->running(), INTAKE);
}

TEST(RollerIntake, IntakeGoingUp) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeStatusProto status;
  GroundBallIntakeGoalProto goal;
  DriverStationStatus ds_status;
  ds_status.set_mode(RobotMode::TELEOP);
  RollerGoal roller_goal = RollerGoal::INTAKE;
  goal->set_intake_up(true);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  intake.SetGoal(goal);
  output = intake.Update(ds_status);
  status = intake.get_status();
  EXPECT_NEAR(output->roller_voltage(), 12, 1e-5);
  EXPECT_TRUE(output->intake_up());
  EXPECT_TRUE(status->is_intake_up());
  EXPECT_EQ(status->running(), INTAKE);
}

TEST(RollerOuttake, IntakeGoingDown) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeStatusProto status;
  GroundBallIntakeGoalProto goal;
  DriverStationStatus ds_status;
  ds_status.set_mode(RobotMode::TELEOP);
  RollerGoal roller_goal = RollerGoal::OUTTAKE;
  goal->set_intake_up(false);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  intake.SetGoal(goal);
  output = intake.Update(ds_status);
  status = intake.get_status();
  EXPECT_NEAR(output->roller_voltage(), -12, 1e-5);
  EXPECT_FALSE(output->intake_up());
  EXPECT_FALSE(status->is_intake_up());
  EXPECT_EQ(status->running(), OUTTAKE);
}

TEST(RollerOuttake, IntakeGoingUp) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeStatusProto status;
  GroundBallIntakeGoalProto goal;
  DriverStationStatus ds_status;
  ds_status.set_mode(RobotMode::TELEOP);
  RollerGoal roller_goal = RollerGoal::OUTTAKE;
  goal->set_intake_up(true);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  intake.SetGoal(goal);
  output = intake.Update(ds_status);
  status = intake.get_status();
  EXPECT_NEAR(output->roller_voltage(), -12, 1e-5);
  EXPECT_TRUE(output->intake_up());
  EXPECT_TRUE(status->is_intake_up());
  EXPECT_EQ(status->running(), OUTTAKE);
}

TEST(RollerIdle, IntakeGoingDown) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeStatusProto status;
  GroundBallIntakeGoalProto goal;
  DriverStationStatus ds_status;
  ds_status.set_mode(RobotMode::TELEOP);
  RollerGoal roller_goal = RollerGoal::IDLE;
  goal->set_intake_up(false);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  intake.SetGoal(goal);
  output = intake.Update(ds_status);
  status = intake.get_status();
  EXPECT_NEAR(output->roller_voltage(), 0, 1e-5);
  EXPECT_FALSE(output->intake_up());
  EXPECT_FALSE(status->is_intake_up());
  EXPECT_EQ(status->running(), IDLE);
}

TEST(RollerIdle, IntakeGoingUp) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeStatusProto status;
  GroundBallIntakeGoalProto goal;
  DriverStationStatus ds_status;
  ds_status.set_mode(RobotMode::TELEOP);
  RollerGoal roller_goal = RollerGoal::IDLE;
  goal->set_intake_up(true);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  intake.SetGoal(goal);
  output = intake.Update(ds_status);
  status = intake.get_status();
  EXPECT_NEAR(output->roller_voltage(), 0, 1e-5);
  EXPECT_TRUE(output->intake_up());
  EXPECT_TRUE(status->is_intake_up());
  EXPECT_EQ(status->running(), IDLE);
}
