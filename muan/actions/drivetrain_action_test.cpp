#include "muan/actions/drivetrain_action.h"
#include "gtest/gtest.h"
#include <memory>

using namespace muan::actions;

class DrivetrainActionTest : public ::testing::Test {
 protected:
  frc971::control_loops::drivetrain::GoalQueue goal_queue;
  frc971::control_loops::drivetrain::StatusQueue status_queue;

  void WriteStatus(double left, double right) {
    frc971::control_loops::drivetrain::StatusProto status;
    status->set_forward_velocity(0);
    status->set_estimated_left_position(left);
    status->set_estimated_right_position(right);
    status->set_estimated_left_velocity(0);
    status->set_estimated_right_velocity(0);
    status->set_profiled_left_position_goal(0);
    status->set_profiled_right_position_goal(0);
    status_queue.WriteMessage(status);
  }

  void CheckGoal(double left, double right) {
    auto maybe_goal = goal_queue.MakeReader().ReadLastMessage();
    EXPECT_TRUE(maybe_goal);
    auto goal = maybe_goal.value();
    EXPECT_TRUE(goal->has_distance_command());
    EXPECT_EQ(goal->distance_command().left_goal(), left);
    EXPECT_EQ(goal->distance_command().right_goal(), right);
    EXPECT_EQ(goal->distance_command().left_velocity_goal(), 0.0);
    EXPECT_EQ(goal->distance_command().right_velocity_goal(), 0.0);
  }
};

DrivetrainProperties properties{1.0, 1.0, 1.0, 1.0, 0.5};

TEST_F(DrivetrainActionTest, DriveStraight) {
  auto action = DrivetrainAction::DriveStraight(1.0, true, properties,
                                                &goal_queue, &status_queue);
  EXPECT_TRUE(action.Update());
  CheckGoal(1, 1);
  WriteStatus(1, 1);
  EXPECT_FALSE(action.Update());
}

TEST_F(DrivetrainActionTest, InitialOffset) {
  WriteStatus(0, 1);
  auto action = DrivetrainAction::DriveStraight(1.0, true, properties,
                                                &goal_queue, &status_queue);
  EXPECT_TRUE(action.Update());
  CheckGoal(1, 2);
  WriteStatus(1, 2);
  EXPECT_FALSE(action.Update());
}

TEST_F(DrivetrainActionTest, PointTurn) {
  WriteStatus(0, 0);
  auto action = DrivetrainAction::PointTurn(1.0, true, properties, &goal_queue,
                                            &status_queue);
  EXPECT_TRUE(action.Update());
  CheckGoal(-0.5, 0.5);
  WriteStatus(-0.5, 0.5);
  EXPECT_FALSE(action.Update());
}

TEST_F(DrivetrainActionTest, SwoopTurn) {
  {
    frc971::control_loops::drivetrain::StatusProto status;
    status->set_forward_velocity(0);
    status->set_estimated_left_position(0);
    status->set_estimated_right_position(0);
    status->set_estimated_left_velocity(0);
    status->set_estimated_right_velocity(0);
    status->set_profiled_left_position_goal(0);
    status->set_profiled_right_position_goal(0);
    status_queue.WriteMessage(status);
  }
  // Swoop turn 1m at 1rad
  auto action = DrivetrainAction::SwoopTurn(2.0, 1.0, true, properties,
                                            &goal_queue, &status_queue);
  EXPECT_TRUE(action.Update());
  {
    auto maybe_goal = goal_queue.MakeReader().ReadLastMessage();
    EXPECT_TRUE(maybe_goal);
    auto goal = maybe_goal.value();
    EXPECT_TRUE(goal->has_distance_command());

    EXPECT_EQ(goal->angular_constraints().max_velocity() * 2,
              goal->linear_constraints().max_velocity());
    EXPECT_EQ(goal->angular_constraints().max_acceleration() * 2,
              goal->linear_constraints().max_acceleration());
  }
  WriteStatus(1.5, 2.5);
  EXPECT_FALSE(action.Update());
}

TEST_F(DrivetrainActionTest, SwoopTurnClockwise) {
  WriteStatus(0, 0);
  // Swoop turn 1m at 1rad
  auto action = DrivetrainAction::SwoopTurn(2.0, -1.0, true, properties,
                                            &goal_queue, &status_queue);
  EXPECT_TRUE(action.Update());
  {
    auto maybe_goal = goal_queue.MakeReader().ReadLastMessage();
    EXPECT_TRUE(maybe_goal);
    auto goal = maybe_goal.value();
    EXPECT_TRUE(goal->has_distance_command());

    EXPECT_EQ(goal->angular_constraints().max_velocity() * 2,
              goal->linear_constraints().max_velocity());
    EXPECT_EQ(goal->angular_constraints().max_acceleration() * 2,
              goal->linear_constraints().max_acceleration());
  }
  WriteStatus(2.5, 1.5);
  EXPECT_FALSE(action.Update());
}

TEST_F(DrivetrainActionTest, SwoopTurnBackwards) {
  WriteStatus(0, 0);
  // Swoop turn 1m at 1rad
  auto action = DrivetrainAction::SwoopTurn(-2.0, 1.0, true, properties,
                                            &goal_queue, &status_queue);
  EXPECT_TRUE(action.Update());
  {
    auto maybe_goal = goal_queue.MakeReader().ReadLastMessage();
    EXPECT_TRUE(maybe_goal);
    auto goal = maybe_goal.value();
    EXPECT_TRUE(goal->has_distance_command());

    EXPECT_EQ(goal->angular_constraints().max_velocity() * 2,
              goal->linear_constraints().max_velocity());
    EXPECT_EQ(goal->angular_constraints().max_acceleration() * 2,
              goal->linear_constraints().max_acceleration());
  }
  WriteStatus(-2.5, -1.5);
  EXPECT_FALSE(action.Update());
}
