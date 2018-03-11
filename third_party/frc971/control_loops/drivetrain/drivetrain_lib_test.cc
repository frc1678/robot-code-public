#include "gtest/gtest.h"
#include "third_party/frc971/control_loops/drivetrain/drivetrain_simulation.h"

#include "third_party/aos/common/controls/polytope.h"
#include "third_party/aos/common/time.h"
#include "third_party/frc971/control_loops/coerce_goal.h"
#include "third_party/frc971/control_loops/drivetrain/queue_types.h"
#include "third_party/frc971/control_loops/drivetrain/y2016/drivetrain_dog_motor_plant.h"
#include "third_party/frc971/control_loops/drivetrain/y2016/drivetrain_base.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {
namespace testing {

using ::third_party::frc971::control_loops::drivetrain::y2016::
    MakeDrivetrainPlant;

using ::y2016::control_loops::drivetrain::GetDrivetrainConfig;

class Y2016DrivetrainTest : public DrivetrainTest {
 public:
  Y2016DrivetrainTest()
      : DrivetrainTest(GetDrivetrainConfig(), MakeDrivetrainPlant()) {}
};

// Tests that the drivetrain converges on a goal.
TEST_F(Y2016DrivetrainTest, ConvergesCorrectly) {
  ::frc971::control_loops::drivetrain::GoalProto goal;
  goal->mutable_distance_command()->set_left_goal(-1.0);
  goal->mutable_distance_command()->set_right_goal(1.0);

  goal_queue_->WriteMessage(goal);

  RunForTime(::std::chrono::seconds(2));

  VerifyNearGoal();
}

// Tests that the drivetrain converges on a goal when under the effect of a
// voltage offset/disturbance.
TEST_F(Y2016DrivetrainTest, ConvergesWithVoltageError) {
  ::frc971::control_loops::drivetrain::GoalProto goal;
  goal->mutable_distance_command()->set_left_goal(-1.0);
  goal->mutable_distance_command()->set_right_goal(1.0);

  goal_queue_->WriteMessage(goal);

  drivetrain_motor_plant_.set_left_voltage_offset(1.0);
  drivetrain_motor_plant_.set_right_voltage_offset(1.0);
  RunForTime(::std::chrono::milliseconds(1500));
  VerifyNearGoal();
}

// Tests that it survives disabling.
TEST_F(Y2016DrivetrainTest, SurvivesDisabling) {
  ::frc971::control_loops::drivetrain::GoalProto goal;
  goal->mutable_distance_command()->set_left_goal(-1.0);
  goal->mutable_distance_command()->set_right_goal(1.0);

  goal_queue_->WriteMessage(goal);

  for (int i = 0; i < 500; ++i) {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Update();
    drivetrain_motor_plant_.Simulate();
    if (i > 20 && i < 200) {
      SimulateTimestep(false);
    } else {
      SimulateTimestep(true);
    }

    // It takes one cycle for disabling to set in
    if (i > 21 && i < 201) {
      // It's disabled, so everything should be 0
      auto output = output_queue_->MakeReader().ReadLastMessage();
      ASSERT_TRUE(output);
      EXPECT_NEAR((*output)->left_voltage(), 0, 1e-7);
      EXPECT_NEAR((*output)->right_voltage(), 0, 1e-7);
    }
  }
  VerifyNearGoal();
}

// Tests that never having a goal doesn't break.
TEST_F(Y2016DrivetrainTest, NoGoalStart) {
  for (int i = 0; i < 20; ++i) {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Update();
    drivetrain_motor_plant_.Simulate();
  }
}

// Tests that never having a goal, but having driver's station messages, doesn't
// break.
TEST_F(Y2016DrivetrainTest, NoGoalWithRobotState) {
  RunForTime(::std::chrono::milliseconds(100));
}

// Tests that the robot successfully drives straight forward.
// This used to not work due to a U-capping bug.
TEST_F(Y2016DrivetrainTest, DriveStraightForward) {
  ::frc971::control_loops::drivetrain::GoalProto goal;
  goal->mutable_distance_command()->set_left_goal(4.0);
  goal->mutable_distance_command()->set_right_goal(4.0);

  goal_queue_->WriteMessage(goal);

  for (int i = 0; i < 600; ++i) {
    RunIteration();
    auto output = output_queue_->MakeReader().ReadLastMessage();
    ASSERT_TRUE(output);
    EXPECT_NEAR((*output)->left_voltage(), (*output)->right_voltage(), 1e-2);
    EXPECT_GE((*output)->left_voltage(), -12.001);
    EXPECT_GE((*output)->right_voltage(), -12.001);
  }
  VerifyNearGoal();
}

// Tests that the robot successfully drives close to straight.
// This used to fail in simulation due to libcdd issues with U-capping.
TEST_F(Y2016DrivetrainTest, DriveAlmostStraightForward) {
  ::frc971::control_loops::drivetrain::GoalProto goal;
  goal->mutable_distance_command()->set_left_goal(4.0);
  goal->mutable_distance_command()->set_right_goal(3.9);

  goal_queue_->WriteMessage(goal);

  for (int i = 0; i < 600; ++i) {
    RunIteration();
    auto output = output_queue_->MakeReader().ReadLastMessage();
    ASSERT_TRUE(output);
    EXPECT_GT((*output)->left_voltage(), -12.001 * kVoltageScale);
    EXPECT_GT((*output)->right_voltage(), -12.001 * kVoltageScale);
  }
  VerifyNearGoal();
}

// Tests that converting from a left, right position to a distance, angle
// coordinate system and back returns the same answer.
TEST_F(Y2016DrivetrainTest, LinearToAngularAndBack) {
  StateFeedbackLoop<7, 2, 3> kf(
      GetDrivetrainConfig().make_kf_drivetrain_loop());
  double kf_heading = 0;
  DrivetrainMotorsSS drivetrain_ss(GetDrivetrainConfig(), &kf, &kf_heading, nullptr);

  const double width = GetDrivetrainConfig().robot_radius * 2.0;

  Eigen::Matrix<double, 7, 1> state;
  state << 2, 3, 4, 5, 0, 0, 0;
  Eigen::Matrix<double, 2, 1> linear = drivetrain_ss.LeftRightToLinear(state);

  EXPECT_NEAR(3.0, linear(0, 0), 1e-6);
  EXPECT_NEAR(4.0, linear(1, 0), 1e-6);

  Eigen::Matrix<double, 2, 1> angular = drivetrain_ss.LeftRightToAngular(state);

  EXPECT_NEAR(2.0 / width, angular(0, 0), 1e-6);
  EXPECT_NEAR(2.0 / width, angular(1, 0), 1e-6);

  Eigen::Matrix<double, 4, 1> back_state =
      drivetrain_ss.AngularLinearToLeftRight(linear, angular);

  for (int i = 0; i < 4; ++i) {
    EXPECT_NEAR(state(i, 0), back_state(i, 0), 1e-8);
  }
}

// Tests that a linear motion profile succeeds.
TEST_F(Y2016DrivetrainTest, ProfileStraightForward) {
  {
    ::frc971::control_loops::drivetrain::GoalProto goal;
    goal->mutable_distance_command()->set_left_goal(4.0);
    goal->mutable_distance_command()->set_right_goal(4.0);
    goal->mutable_distance_command()->set_left_velocity_goal(0.0);
    goal->mutable_distance_command()->set_right_velocity_goal(0.0);
    goal->mutable_linear_constraints()->set_max_velocity(1.0);
    goal->mutable_linear_constraints()->set_max_acceleration(3.0);
    goal->mutable_angular_constraints()->set_max_velocity(1.0);
    goal->mutable_angular_constraints()->set_max_acceleration(3.0);
    goal_queue_->WriteMessage(goal);
  }

  while (aos::time::Time::Now() < aos::time::Time::InSeconds(6)) {
    RunIteration();

    auto output = output_queue_->MakeReader().ReadLastMessage();
    ASSERT_TRUE(output);
    EXPECT_GT((*output)->left_voltage(), -12.001 * kVoltageScale);
    EXPECT_GT((*output)->right_voltage(), -12.001 * kVoltageScale);
    EXPECT_NEAR((*output)->left_voltage(), (*output)->right_voltage(), 1);
    EXPECT_GT((*output)->left_voltage(), -12.001 * kVoltageScale);
    EXPECT_GT((*output)->right_voltage(), -12.001 * kVoltageScale);
    EXPECT_LT((*output)->left_voltage(), 12.001 * kVoltageScale);
    EXPECT_LT((*output)->right_voltage(), 12.001 * kVoltageScale);
  }
  VerifyNearGoal();
}

// Tests that an angular motion profile succeeds.
TEST_F(Y2016DrivetrainTest, ProfileTurn) {
  {
    ::frc971::control_loops::drivetrain::GoalProto goal;
    goal->mutable_distance_command()->set_left_goal(-1.0);
    goal->mutable_distance_command()->set_right_goal(1.0);
    goal->mutable_distance_command()->set_left_velocity_goal(0.0);
    goal->mutable_distance_command()->set_right_velocity_goal(0.0);
    goal->mutable_linear_constraints()->set_max_velocity(1.0);
    goal->mutable_linear_constraints()->set_max_acceleration(3.0);
    goal->mutable_angular_constraints()->set_max_velocity(1.0);
    goal->mutable_angular_constraints()->set_max_acceleration(3.0);
    goal_queue_->WriteMessage(goal);
  }

  while (aos::time::Time::Now() < aos::time::Time::InSeconds(6)) {
    RunIteration();

    auto output = output_queue_->MakeReader().ReadLastMessage();
    ASSERT_TRUE(output);
    EXPECT_GT((*output)->left_voltage(), -12.001 * kVoltageScale);
    EXPECT_GT((*output)->right_voltage(), -12.001 * kVoltageScale);
    EXPECT_NEAR((*output)->left_voltage(), -(*output)->right_voltage(), 3);
    EXPECT_GT((*output)->left_voltage(), -12.001 * kVoltageScale);
    EXPECT_GT((*output)->right_voltage(), -12.001 * kVoltageScale);
    EXPECT_LT((*output)->left_voltage(), 12.001 * kVoltageScale);
    EXPECT_LT((*output)->right_voltage(), 12.001 * kVoltageScale);
  }
  VerifyNearGoal();
}

// Tests that a mixed turn drive saturated profile succeeds.
TEST_F(Y2016DrivetrainTest, SaturatedTurnDrive) {
  {
    ::frc971::control_loops::drivetrain::GoalProto goal;
    goal->mutable_distance_command()->set_left_goal(5.0);
    goal->mutable_distance_command()->set_right_goal(4.0);
    goal->mutable_distance_command()->set_left_velocity_goal(0.0);
    goal->mutable_distance_command()->set_right_velocity_goal(0.0);
    goal->mutable_linear_constraints()->set_max_velocity(6.0);
    goal->mutable_linear_constraints()->set_max_acceleration(4.0);
    goal->mutable_angular_constraints()->set_max_velocity(2.0);
    goal->mutable_angular_constraints()->set_max_acceleration(4.0);
    goal_queue_->WriteMessage(goal);
  }

  while (aos::time::Time::Now() < aos::time::Time::InSeconds(3.5)) {
    RunIteration();
    ASSERT_TRUE(output_queue_->MakeReader().ReadLastMessage());
  }
  VerifyNearGoal();
}

// Tests that being in teleop drive for a bit and then transitioning to closed
// drive profiles nicely.
TEST_F(Y2016DrivetrainTest, OpenLoopThenClosed) {
  {
    ::frc971::control_loops::drivetrain::GoalProto goal;
    goal->mutable_teleop_command()->set_closed_loop(true);
    goal->mutable_teleop_command()->set_steering(0.0);
    goal->mutable_teleop_command()->set_throttle(1.0);
    goal->set_gear(Gear::kHighGear);
    goal->mutable_teleop_command()->set_quick_turn(false);
    goal_queue_->WriteMessage(goal);
  }

  RunForTime(::std::chrono::milliseconds(1000));

  {
    ::frc971::control_loops::drivetrain::GoalProto goal;
    goal->mutable_teleop_command()->set_closed_loop(true);
    goal->mutable_teleop_command()->set_steering(0.0);
    goal->mutable_teleop_command()->set_throttle(-0.3);
    goal->set_gear(Gear::kHighGear);
    goal->mutable_teleop_command()->set_quick_turn(false);
    goal_queue_->WriteMessage(goal);
  }

  RunForTime(::std::chrono::milliseconds(1000));

  {
    ::frc971::control_loops::drivetrain::GoalProto goal;
    goal->mutable_teleop_command()->set_closed_loop(true);
    goal->mutable_teleop_command()->set_steering(0.0);
    goal->mutable_teleop_command()->set_throttle(0.0);
    goal->set_gear(Gear::kHighGear);
    goal->mutable_teleop_command()->set_quick_turn(false);
    goal_queue_->WriteMessage(goal);
  }

  RunForTime(::std::chrono::milliseconds(10000));

  {
    ::frc971::control_loops::drivetrain::GoalProto goal;
    goal->mutable_distance_command()->set_left_goal(5.0);
    goal->mutable_distance_command()->set_right_goal(4.0);
    goal->mutable_distance_command()->set_left_velocity_goal(0.0);
    goal->mutable_distance_command()->set_left_velocity_goal(0.0);
    goal->mutable_linear_constraints()->set_max_velocity(1.0);
    goal->mutable_linear_constraints()->set_max_acceleration(2.0);
    goal->mutable_angular_constraints()->set_max_velocity(1.0);
    goal->mutable_angular_constraints()->set_max_acceleration(2.0);
    goal_queue_->WriteMessage(goal);
  }

  const auto end_time = aos::time::Time::Now() + aos::time::Time::InSeconds(4);
  while (aos::time::Time::Now() < end_time) {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Update();
    drivetrain_motor_plant_.Simulate();
    SimulateTimestep(true);

    auto output = output_queue_->MakeReader().ReadLastMessage();
    ASSERT_TRUE(output);
    EXPECT_GT((*output)->left_voltage(), -12.001 * kVoltageScale);
    EXPECT_GT((*output)->right_voltage(), -12.001 * kVoltageScale);
    EXPECT_LT((*output)->left_voltage(), 12.001 * kVoltageScale);
    EXPECT_LT((*output)->right_voltage(), 12.001 * kVoltageScale);
  }
  VerifyNearGoal();
}


TEST_F(Y2016DrivetrainTest, CartesianEstimation) {
  ::frc971::control_loops::drivetrain::GoalProto goal;
  goal->mutable_distance_command()->set_left_goal(2.0);
  goal->mutable_distance_command()->set_right_goal(2.0);

  goal_queue_->WriteMessage(goal);

  RunForTime(::std::chrono::seconds(6));

  double r = ::third_party::frc971::control_loops::drivetrain::y2016::kRobotRadius;
  goal->mutable_distance_command()->set_left_goal(2.0 - r * M_PI / 2.0);
  goal->mutable_distance_command()->set_right_goal(2.0 + r * M_PI / 2.0);

  goal_queue_->WriteMessage(goal);

  RunForTime(::std::chrono::seconds(6));
  VerifyNearGoal();

  goal->mutable_distance_command()->set_left_goal(2.0 - r * M_PI / 2.0 + 1.0);
  goal->mutable_distance_command()->set_right_goal(2.0 + r * M_PI / 2.0 + 1.0);

  goal_queue_->WriteMessage(goal);

  RunForTime(::std::chrono::seconds(5));

  {
    auto maybe_status = status_queue_->ReadLastMessage();
    ASSERT_TRUE(maybe_status);
    ::frc971::control_loops::drivetrain::StatusProto status = *maybe_status;
    EXPECT_NEAR(status->estimated_x_position(), 2.0, 1e-2);
    EXPECT_NEAR(status->estimated_y_position(), 1.0, 1e-2);
  }
}

TEST_F(Y2016DrivetrainTest, PathDrive) {
  ::frc971::control_loops::drivetrain::GoalProto goal;

  goal->mutable_path_command()->set_x_goal(1.0);
  goal->mutable_path_command()->set_y_goal(1.0);
  goal->mutable_path_command()->set_theta_goal(0.0);

  goal->mutable_linear_constraints()->set_max_velocity(2.0);
  goal->mutable_linear_constraints()->set_max_acceleration(2.0);
  goal->mutable_angular_constraints()->set_max_velocity(2.0);
  goal->mutable_angular_constraints()->set_max_acceleration(2.0);

  goal_queue_->WriteMessage(goal);
  RunForTime(::std::chrono::seconds(10));
  {
    auto maybe_status = status_queue_->ReadLastMessage();
    ASSERT_TRUE(maybe_status);
    ::frc971::control_loops::drivetrain::StatusProto status = *maybe_status;
    EXPECT_NEAR(status->estimated_x_position(), 1.0, 1e-1);
    EXPECT_NEAR(status->estimated_y_position(), 1.0, 1e-1);
    EXPECT_NEAR(status->estimated_heading(), 0.0, 1e-2);
  }
}

TEST_F(Y2016DrivetrainTest, TransitionToPath) {
  ::frc971::control_loops::drivetrain::GoalProto goal;

  goal->mutable_distance_command()->set_left_goal(5.25);
  goal->mutable_distance_command()->set_right_goal(5.25);
  goal->mutable_linear_constraints()->set_max_velocity(3.0);
  goal->mutable_linear_constraints()->set_max_acceleration(3.0);
  goal->mutable_angular_constraints()->set_max_velocity(3.0);
  goal->mutable_angular_constraints()->set_max_acceleration(3.0);
  goal_queue_->WriteMessage(goal);
  RunForTime(::std::chrono::milliseconds(1600));

  goal->mutable_path_command()->set_x_goal(2.2);
  goal->mutable_path_command()->set_y_goal(3.62);
  goal->mutable_path_command()->set_theta_goal(-0.628);
  goal->mutable_linear_constraints()->set_max_velocity(3.0);
  goal->mutable_linear_constraints()->set_max_acceleration(3.0);
  goal->mutable_angular_constraints()->set_max_velocity(3.0);
  goal->mutable_angular_constraints()->set_max_acceleration(3.0);
  goal_queue_->WriteMessage(goal);
  RunForTime(::std::chrono::seconds(10));

  {
    auto maybe_status = status_queue_->ReadLastMessage();
    ASSERT_TRUE(maybe_status);
    ::frc971::control_loops::drivetrain::StatusProto status = *maybe_status;
    EXPECT_NEAR(status->estimated_x_position(), 2.2, 1e-1);
    EXPECT_NEAR(status->estimated_y_position(), 3.62, 1e-1);
    EXPECT_NEAR(status->estimated_heading(), M_PI * 2 - 0.628, 1e-2);
  }
}

::aos::controls::HVPolytope<2, 4, 4> MakeBox(double x1_min, double x1_max,
                                             double x2_min, double x2_max) {
  Eigen::Matrix<double, 4, 2> box_H;
  box_H << /*[[*/ 1.0, 0.0 /*]*/,
      /*[*/ -1.0, 0.0 /*]*/,
      /*[*/ 0.0, 1.0 /*]*/,
      /*[*/ 0.0, -1.0 /*]]*/;
  Eigen::Matrix<double, 4, 1> box_k;
  box_k << /*[[*/ x1_max /*]*/,
      /*[*/ -x1_min /*]*/,
      /*[*/ x2_max /*]*/,
      /*[*/ -x2_min /*]]*/;
  ::aos::controls::HPolytope<2> t_poly(box_H, box_k);
  return ::aos::controls::HVPolytope<2, 4, 4>(t_poly.H(), t_poly.k(),
                                              t_poly.Vertices());
}

class CoerceGoalTest : public ::testing::Test {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// WHOOOHH!
TEST_F(CoerceGoalTest, Inside) {
  ::aos::controls::HVPolytope<2, 4, 4> box = MakeBox(1, 2, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << /*[[*/ 1, -1 /*]]*/;

  Eigen::Matrix<double, 2, 1> R;
  R << /*[[*/ 1.5, 1.5 /*]]*/;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal(box, K, 0, R);

  EXPECT_EQ(R(0, 0), output(0, 0));
  EXPECT_EQ(R(1, 0), output(1, 0));
}

TEST_F(CoerceGoalTest, Outside_Inside_Intersect) {
  ::aos::controls::HVPolytope<2, 4, 4> box = MakeBox(1, 2, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << 1, -1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal(box, K, 0, R);

  EXPECT_EQ(2.0, output(0, 0));
  EXPECT_EQ(2.0, output(1, 0));
}

TEST_F(CoerceGoalTest, Outside_Inside_no_Intersect) {
  ::aos::controls::HVPolytope<2, 4, 4> box = MakeBox(3, 4, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << 1, -1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal(box, K, 0, R);

  EXPECT_EQ(3.0, output(0, 0));
  EXPECT_EQ(2.0, output(1, 0));
}

TEST_F(CoerceGoalTest, Middle_Of_Edge) {
  ::aos::controls::HVPolytope<2, 4, 4> box = MakeBox(0, 4, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << -1, 1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal(box, K, 0, R);

  EXPECT_EQ(2.0, output(0, 0));
  EXPECT_EQ(2.0, output(1, 0));
}

TEST_F(CoerceGoalTest, PerpendicularLine) {
  ::aos::controls::HVPolytope<2, 4, 4> box = MakeBox(1, 2, 1, 2);

  Eigen::Matrix<double, 1, 2> K;
  K << 1, 1;

  Eigen::Matrix<double, 2, 1> R;
  R << 5, 5;

  Eigen::Matrix<double, 2, 1> output =
      ::frc971::control_loops::CoerceGoal(box, K, 0, R);

  EXPECT_EQ(1.0, output(0, 0));
  EXPECT_EQ(1.0, output(1, 0));
}

// TODO(austin): Make sure the profile reset code when we disable works.

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
