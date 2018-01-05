#include "muan/actions/drivetrain_action.h"
#include <memory>
#include "gtest/gtest.h"

using namespace muan::actions;  // NOLINT

class DrivetrainActionTest : public ::testing::Test {
 protected:
  frc971::control_loops::drivetrain::GoalQueue goal_queue{100};
  frc971::control_loops::drivetrain::StatusQueue status_queue{100};

  void WriteStatus(double left, double right, double left_profile = 0.0, double right_profile = 0.0) {
    frc971::control_loops::drivetrain::StatusProto status;
    status->set_forward_velocity(0);
    status->set_estimated_left_position(left);
    status->set_estimated_right_position(right);
    status->set_estimated_left_velocity(0);
    status->set_estimated_right_velocity(0);
    status->set_profiled_left_position_goal(left_profile);
    status->set_profiled_right_position_goal(right_profile);
    status_queue.WriteMessage(status);
  }

  void CheckGoal(double left, double right, double left_velocity = 0.0, double right_velocity = 0.0,
                 double left_acceleration = 1.0, double right_acceleration = 1.0) {
    auto maybe_goal = goal_queue.MakeReader().ReadLastMessage();
    EXPECT_TRUE(maybe_goal);
    auto goal = maybe_goal.value();
    EXPECT_TRUE(goal->has_distance_command());
    EXPECT_EQ(goal->distance_command().left_goal(),
              left + left_velocity * left_velocity / left_acceleration / 2.0);
    EXPECT_EQ(goal->distance_command().right_goal(),
              right + right_velocity * right_velocity / right_acceleration / 2.0);
    EXPECT_EQ(goal->distance_command().left_velocity_goal(), 0);
    EXPECT_EQ(goal->distance_command().right_velocity_goal(), 0);
  }
};

DrivetrainProperties properties{1.0, 1.0, 1.0, 1.0, 0.5};
DrivetrainTermination termination{0.01, 0.01, 0.01, 0.01};

TEST_F(DrivetrainActionTest, DriveStraight) {
  DrivetrainAction action(properties, &goal_queue, &status_queue);
  DrivetrainActionParams params;
  params.desired_forward_distance = 1.0;
  action.ExecuteDrive(params);

  EXPECT_TRUE(action.Update());
  CheckGoal(1, 1);
  WriteStatus(1, 1);
  EXPECT_FALSE(action.Update());
}

TEST_F(DrivetrainActionTest, InitialOffset) {
  DrivetrainAction action(properties, &goal_queue, &status_queue);

  WriteStatus(0, 1);

  DrivetrainActionParams params;
  params.desired_forward_distance = 1.0;

  action.ExecuteDrive(params);

  EXPECT_TRUE(action.Update());
  CheckGoal(1, 2);
  WriteStatus(1, 2);
  EXPECT_FALSE(action.Update());
}

TEST_F(DrivetrainActionTest, Backwards) {
  DrivetrainAction action(properties, &goal_queue, &status_queue);

  WriteStatus(0, 1);

  DrivetrainActionParams params;
  params.desired_forward_distance = -1.0;

  action.ExecuteDrive(params);

  EXPECT_TRUE(action.Update());
  CheckGoal(-1, 0);
  WriteStatus(-1, 0);
  EXPECT_FALSE(action.Update());
}

TEST_F(DrivetrainActionTest, PointTurn) {
  DrivetrainAction action(properties, &goal_queue, &status_queue);

  WriteStatus(0, 0);

  DrivetrainActionParams params;
  params.desired_angular_displacement = 1.0;

  action.ExecuteDrive(params);

  EXPECT_TRUE(action.Update());
  CheckGoal(-0.5, 0.5);
  WriteStatus(-0.5, 0.5);
  EXPECT_FALSE(action.Update());
}

TEST_F(DrivetrainActionTest, PointTurnClockwise) {
  DrivetrainAction action(properties, &goal_queue, &status_queue);

  WriteStatus(0, 0);

  DrivetrainActionParams params;
  params.desired_angular_displacement = -1.0;

  action.ExecuteDrive(params);

  EXPECT_TRUE(action.Update());
  CheckGoal(0.5, -0.5);
  WriteStatus(0.5, -0.5);
  EXPECT_FALSE(action.Update());
}

TEST_F(DrivetrainActionTest, SwoopTurn) {
  DrivetrainAction action(properties, &goal_queue, &status_queue);

  WriteStatus(0.0, 0.0);

  DrivetrainActionParams params;
  params.desired_angular_displacement = 1.0;
  params.desired_forward_distance = 2.0;

  action.ExecuteDrive(params);

  EXPECT_TRUE(action.Update());
  CheckGoal(1.5, 2.5);
  {
    auto maybe_goal = goal_queue.MakeReader().ReadLastMessage();
    EXPECT_TRUE(maybe_goal);

    auto goal = maybe_goal.value();

    EXPECT_TRUE(goal->has_distance_command());

    EXPECT_EQ(goal->angular_constraints().max_velocity() * 2, goal->linear_constraints().max_velocity());
    EXPECT_EQ(goal->angular_constraints().max_acceleration() * 2,
              goal->linear_constraints().max_acceleration());
  }
  WriteStatus(1.5, 2.5);
  EXPECT_FALSE(action.Update());
}

TEST_F(DrivetrainActionTest, SwoopTurnBackwards) {
  DrivetrainAction action(properties, &goal_queue, &status_queue);

  WriteStatus(0.0, 0.0);

  DrivetrainActionParams params;
  params.desired_angular_displacement = 1.0;
  params.desired_forward_distance = -2.0;

  action.ExecuteDrive(params);

  EXPECT_TRUE(action.Update());
  CheckGoal(-2.5, -1.5);
  {
    auto maybe_goal = goal_queue.MakeReader().ReadLastMessage();
    EXPECT_TRUE(maybe_goal);

    auto goal = maybe_goal.value();

    EXPECT_TRUE(goal->has_distance_command());

    EXPECT_EQ(goal->angular_constraints().max_velocity() * 2, goal->linear_constraints().max_velocity());
    EXPECT_EQ(goal->angular_constraints().max_acceleration() * 2,
              goal->linear_constraints().max_acceleration());
  }
  WriteStatus(-2.5, -1.5);
  EXPECT_FALSE(action.Update());
}

TEST_F(DrivetrainActionTest, SwoopTurnClockwise) {
  DrivetrainAction action(properties, &goal_queue, &status_queue);

  WriteStatus(0.0, 0.0);

  DrivetrainActionParams params;
  params.desired_angular_displacement = -1.0;
  params.desired_forward_distance = 2.0;

  action.ExecuteDrive(params);

  EXPECT_TRUE(action.Update());
  CheckGoal(2.5, 1.5);
  {
    auto maybe_goal = goal_queue.MakeReader().ReadLastMessage();
    EXPECT_TRUE(maybe_goal);

    auto goal = maybe_goal.value();

    EXPECT_TRUE(goal->has_distance_command());

    EXPECT_EQ(goal->angular_constraints().max_velocity() * 2, goal->linear_constraints().max_velocity());
    EXPECT_EQ(goal->angular_constraints().max_acceleration() * 2,
              goal->linear_constraints().max_acceleration());
  }
  WriteStatus(2.5, 1.5);
  EXPECT_FALSE(action.Update());
}

TEST_F(DrivetrainActionTest, FollowThrough) {
  DrivetrainAction action(properties, &goal_queue, &status_queue);

  WriteStatus(0, 0);

  DrivetrainActionParams params;
  params.desired_forward_distance = 1.0;
  params.follow_through = true;
  params.closed_loop_termination = false;

  action.ExecuteDrive(params);

  EXPECT_TRUE(action.Update());
  CheckGoal(1.0, 1.0, 1.0, 1.0, 1.0, 1.0);
  WriteStatus(0.0, 0.0, 1.0, 1.0);
  EXPECT_FALSE(action.Update());
}
