#include "gtest/gtest.h"
#include "muan/wpilib/queue_types.h"
#include "c2017/subsystems/superstructure/ground_ball_intake/queue_types.h"
#include "c2017/subsystems/superstructure/ground_ball_intake/ground_ball_intake.h"

using namespace c2017::ground_ball_intake;

TEST(TestGroundBallIntake, RollerIntakeGoingDown) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeStatusProto status;
  GroundBallIntakeGoalProto goal;
  DriverStationStatus ds_status;
  ds_status.set_mode(RobotMode::TELEOP);
  RollerGoal roller_goal = RollerGoal::INTAKE;
  goal->set_intake_up(false);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  output = intake.Update(ds_status, goal);
  status = intake.get_status();
  EXPECT_NEAR(output->roller_voltage(), 12, 1e-5);
  EXPECT_FALSE(output->intake_up());
  EXPECT_FALSE(status->is_intake_up());
  EXPECT_EQ(status->running(), INTAKE);
}

TEST(TestGroundBallIntake, RollerIntakeGoingUp) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeStatusProto status;
  GroundBallIntakeGoalProto goal;
  DriverStationStatus ds_status;
  ds_status.set_mode(RobotMode::TELEOP);
  RollerGoal roller_goal = RollerGoal::INTAKE;
  goal->set_intake_up(true);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  output = intake.Update(ds_status, goal);
  status = intake.get_status();
  EXPECT_NEAR(output->roller_voltage(), 12, 1e-5);
  EXPECT_TRUE(output->intake_up());
  EXPECT_TRUE(status->is_intake_up());
  EXPECT_EQ(status->running(), INTAKE);
}

TEST(TestGroundBallIntake, RollerOuttakeGoingDown) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeStatusProto status;
  GroundBallIntakeGoalProto goal;
  DriverStationStatus ds_status;
  ds_status.set_mode(RobotMode::TELEOP);
  RollerGoal roller_goal = RollerGoal::OUTTAKE;
  goal->set_intake_up(false);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  output = intake.Update(ds_status, goal);
  status = intake.get_status();
  EXPECT_NEAR(output->roller_voltage(), -12, 1e-5);
  EXPECT_FALSE(output->intake_up());
  EXPECT_FALSE(status->is_intake_up());
  EXPECT_EQ(status->running(), OUTTAKE);
}

TEST(TestGroundBallIntake, RollerOuttakeGoingUp) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeStatusProto status;
  GroundBallIntakeGoalProto goal;
  DriverStationStatus ds_status;
  ds_status.set_mode(RobotMode::TELEOP);
  RollerGoal roller_goal = RollerGoal::OUTTAKE;
  goal->set_intake_up(true);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  output = intake.Update(ds_status, goal);
  status = intake.get_status();
  EXPECT_NEAR(output->roller_voltage(), -12, 1e-5);
  EXPECT_TRUE(output->intake_up());
  EXPECT_TRUE(status->is_intake_up());
  EXPECT_EQ(status->running(), OUTTAKE);
}

TEST(TestGroundBallIntake, RollerIdleGoingDown) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeStatusProto status;
  GroundBallIntakeGoalProto goal;
  DriverStationStatus ds_status;
  ds_status.set_mode(RobotMode::TELEOP);
  RollerGoal roller_goal = RollerGoal::IDLE;
  goal->set_intake_up(false);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  output = intake.Update(ds_status, goal);
  status = intake.get_status();
  EXPECT_NEAR(output->roller_voltage(), 0, 1e-5);
  EXPECT_FALSE(output->intake_up());
  EXPECT_FALSE(status->is_intake_up());
  EXPECT_EQ(status->running(), IDLE);
}

TEST(TestGroundBallIntake, RollerIdleGoingUp) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeStatusProto status;
  GroundBallIntakeGoalProto goal;
  DriverStationStatus ds_status;
  ds_status.set_mode(RobotMode::TELEOP);
  RollerGoal roller_goal = RollerGoal::IDLE;
  goal->set_intake_up(true);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  output = intake.Update(ds_status, goal);
  status = intake.get_status();
  EXPECT_NEAR(output->roller_voltage(), 0, 1e-5);
  EXPECT_TRUE(output->intake_up());
  EXPECT_TRUE(status->is_intake_up());
  EXPECT_EQ(status->running(), IDLE);
}

TEST(TestGroundBallIntake, DISABLED) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeStatusProto status;
  GroundBallIntakeGoalProto goal;
  DriverStationStatus ds_status;
  ds_status.set_mode(RobotMode::DISABLED);
  RollerGoal roller_goal = RollerGoal::INTAKE;
  goal->set_intake_up(false);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  output = intake.Update(ds_status, goal);
  status = intake.get_status();
  EXPECT_NEAR(output->roller_voltage(), 0, 1e-5);
  EXPECT_TRUE(output->intake_up());
  EXPECT_TRUE(status->is_intake_up());
  EXPECT_EQ(status->running(), INTAKE);
}

TEST(TestGroundBallIntake, ESTOP) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeStatusProto status;
  GroundBallIntakeGoalProto goal;
  DriverStationStatus ds_status;
  ds_status.set_mode(RobotMode::ESTOP);
  RollerGoal roller_goal = RollerGoal::INTAKE;
  goal->set_intake_up(false);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  output = intake.Update(ds_status, goal);
  status = intake.get_status();
  EXPECT_NEAR(output->roller_voltage(), 0, 1e-5);
  EXPECT_TRUE(output->intake_up());
  EXPECT_TRUE(status->is_intake_up());
  EXPECT_EQ(status->running(), INTAKE);
}

TEST(TestGroundBallIntake, Brownout) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeStatusProto status;
  GroundBallIntakeGoalProto goal;
  DriverStationStatus ds_status;
  ds_status.set_mode(RobotMode::TELEOP);
  ds_status.set_brownout(true);
  RollerGoal roller_goal = RollerGoal::INTAKE;
  goal->set_intake_up(false);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  output = intake.Update(ds_status, goal);
  status = intake.get_status();
  EXPECT_NEAR(output->roller_voltage(), 0, 1e-5);
  EXPECT_TRUE(output->intake_up());
  EXPECT_TRUE(status->is_intake_up());
  EXPECT_EQ(status->running(), INTAKE);
}
