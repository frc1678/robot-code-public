#include "gtest/gtest.h"
#include "muan/wpilib/queue_types.h"
#include "c2017/subsystems/superstructure/ground_ball_intake/queue_types.h"
#include "c2017/subsystems/superstructure/ground_ball_intake/ground_ball_intake.h"
#include "c2017/queue_manager/queue_manager.h"

using namespace c2017::ground_ball_intake;  // NOLINT

TEST(TestGroundBallIntake, RollerIntakeGoingDown) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeGoalProto goal;
  muan::wpilib::DriverStationProto ds_status;
  ds_status->set_mode(RobotMode::TELEOP);
  ds_status->set_brownout(false);
  RollerGoal roller_goal = RollerGoal::INTAKE;
  goal->set_intake_up(false);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  intake.set_goal(goal);
  output = intake.Update(ds_status);
  auto status = c2017::QueueManager::GetInstance().ground_ball_intake_status_queue().ReadLastMessage();
  EXPECT_NEAR(output->roller_voltage(), 8, 1e-5);
  EXPECT_FALSE(output->intake_up());
  if (status) {
    EXPECT_FALSE(status.value()->is_intake_up());
    EXPECT_EQ(status.value()->running(), RollerGoal::INTAKE);
  } else {
    FAIL();
  }
}

TEST(TestGroundBallIntake, RollerIntakeGoingUp) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeGoalProto goal;
  muan::wpilib::DriverStationProto ds_status;
  ds_status->set_mode(RobotMode::TELEOP);
  RollerGoal roller_goal = RollerGoal::INTAKE;
  goal->set_intake_up(true);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  intake.set_goal(goal);
  output = intake.Update(ds_status);
  auto status = c2017::QueueManager::GetInstance().ground_ball_intake_status_queue().ReadLastMessage();
  EXPECT_NEAR(output->roller_voltage(), 8, 1e-5);
  EXPECT_TRUE(output->intake_up());
  if (status) {
    EXPECT_TRUE(status.value()->is_intake_up());
    EXPECT_EQ(status.value()->running(), RollerGoal::INTAKE);
  } else {
    FAIL();
  }
}

TEST(TestGroundBallIntake, RollerOuttakeGoingDown) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeGoalProto goal;
  muan::wpilib::DriverStationProto ds_status;
  ds_status->set_mode(RobotMode::TELEOP);
  RollerGoal roller_goal = RollerGoal::OUTTAKE;
  goal->set_intake_up(false);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  intake.set_goal(goal);
  output = intake.Update(ds_status);
  auto status = c2017::QueueManager::GetInstance().ground_ball_intake_status_queue().ReadLastMessage();
  EXPECT_NEAR(output->roller_voltage(), -8, 1e-5);
  EXPECT_FALSE(output->intake_up());
  if (status) {
    EXPECT_FALSE(status.value()->is_intake_up());
    EXPECT_EQ(status.value()->running(), RollerGoal::OUTTAKE);
  } else {
    FAIL();
  }
}

TEST(TestGroundBallIntake, RollerOuttakeGoingUp) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeGoalProto goal;
  muan::wpilib::DriverStationProto ds_status;
  ds_status->set_mode(RobotMode::TELEOP);
  RollerGoal roller_goal = RollerGoal::OUTTAKE;
  goal->set_intake_up(true);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  intake.set_goal(goal);
  output = intake.Update(ds_status);
  auto status = c2017::QueueManager::GetInstance().ground_ball_intake_status_queue().ReadLastMessage();
  EXPECT_NEAR(output->roller_voltage(), -8, 1e-5);
  EXPECT_TRUE(output->intake_up());
  if (status) {
    EXPECT_TRUE(status.value()->is_intake_up());
    EXPECT_EQ(status.value()->running(), RollerGoal::OUTTAKE);
  } else {
    FAIL();
  }
}

TEST(TestGroundBallIntake, RollerIdleGoingDown) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeGoalProto goal;
  muan::wpilib::DriverStationProto ds_status;
  ds_status->set_mode(RobotMode::TELEOP);
  RollerGoal roller_goal = RollerGoal::IDLE;
  goal->set_intake_up(false);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  intake.set_goal(goal);
  output = intake.Update(ds_status);
  auto status = c2017::QueueManager::GetInstance().ground_ball_intake_status_queue().ReadLastMessage();
  EXPECT_NEAR(output->roller_voltage(), 0, 1e-5);
  EXPECT_FALSE(output->intake_up());
  if (status) {
    EXPECT_FALSE(status.value()->is_intake_up());
    EXPECT_EQ(status.value()->running(), RollerGoal::IDLE);
  } else {
    FAIL();
  }
}

TEST(TestGroundBallIntake, RollerIdleGoingUp) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeGoalProto goal;
  muan::wpilib::DriverStationProto ds_status;
  ds_status->set_mode(RobotMode::TELEOP);
  RollerGoal roller_goal = RollerGoal::IDLE;
  goal->set_intake_up(true);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  intake.set_goal(goal);
  output = intake.Update(ds_status);
  auto status = c2017::QueueManager::GetInstance().ground_ball_intake_status_queue().ReadLastMessage();
  EXPECT_NEAR(output->roller_voltage(), 0, 1e-5);
  EXPECT_TRUE(output->intake_up());
  if (status) {
    EXPECT_TRUE(status.value()->is_intake_up());
    EXPECT_EQ(status.value()->running(), RollerGoal::IDLE);
  } else {
    FAIL();
  }
}

TEST(TestGroundBallIntake, DISABLED) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeGoalProto goal;
  muan::wpilib::DriverStationProto ds_status;
  ds_status->set_mode(RobotMode::DISABLED);
  RollerGoal roller_goal = RollerGoal::INTAKE;
  goal->set_intake_up(false);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  intake.set_goal(goal);
  output = intake.Update(ds_status);
  auto status = c2017::QueueManager::GetInstance().ground_ball_intake_status_queue().ReadLastMessage();
  EXPECT_NEAR(output->roller_voltage(), 0, 1e-5);
  EXPECT_TRUE(output->intake_up());
  if (status) {
    EXPECT_TRUE(status.value()->is_intake_up());
    EXPECT_EQ(status.value()->running(), RollerGoal::INTAKE);
  } else {
    FAIL();
  }
}

TEST(TestGroundBallIntake, ESTOP) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeGoalProto goal;
  muan::wpilib::DriverStationProto ds_status;
  ds_status->set_mode(RobotMode::ESTOP);
  RollerGoal roller_goal = RollerGoal::INTAKE;
  goal->set_intake_up(false);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  intake.set_goal(goal);
  output = intake.Update(ds_status);
  auto status = c2017::QueueManager::GetInstance().ground_ball_intake_status_queue().ReadLastMessage();
  EXPECT_NEAR(output->roller_voltage(), 0, 1e-5);
  EXPECT_TRUE(output->intake_up());
  if (status) {
    EXPECT_TRUE(status.value()->is_intake_up());
    EXPECT_EQ(status.value()->running(), RollerGoal::INTAKE);
  } else {
    FAIL();
  }
}

TEST(TestGroundBallIntake, Brownout) {
  GroundBallIntakeOutputProto output;
  GroundBallIntakeGoalProto goal;
  muan::wpilib::DriverStationProto ds_status;
  ds_status->set_mode(RobotMode::TELEOP);
  ds_status->set_brownout(true);
  RollerGoal roller_goal = RollerGoal::INTAKE;
  goal->set_intake_up(false);
  goal->set_run_intake(roller_goal);
  GroundBallIntake intake;
  intake.set_goal(goal);
  output = intake.Update(ds_status);
  auto status = c2017::QueueManager::GetInstance().ground_ball_intake_status_queue().ReadLastMessage();
  EXPECT_NEAR(output->roller_voltage(), 0, 1e-5);
  EXPECT_TRUE(output->intake_up());
  if (status) {
    EXPECT_TRUE(status.value()->is_intake_up());
    EXPECT_EQ(status.value()->running(), RollerGoal::INTAKE);
  } else {
    FAIL();
  }
}
