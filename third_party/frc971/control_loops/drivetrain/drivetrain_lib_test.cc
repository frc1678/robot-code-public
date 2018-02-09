#include <unistd.h>

#include <memory>

#include "third_party/aos/common/controls/polytope.h"
#include "third_party/aos/common/time.h"
#include "gtest/gtest.h"

#include "third_party/frc971/control_loops/coerce_goal.h"
#include "third_party/frc971/control_loops/drivetrain/drivetrain.h"
#include "third_party/frc971/control_loops/drivetrain/drivetrain_config.h"
#include "third_party/frc971/control_loops/drivetrain/y2016/drivetrain_dog_motor_plant.h"
#include "third_party/frc971/control_loops/drivetrain/y2016/kalman_drivetrain_motor_plant.h"
#include "third_party/frc971/control_loops/drivetrain/y2016/polydrivetrain_dog_motor_plant.h"
#include "third_party/frc971/control_loops/drivetrain/y2016/drivetrain_base.h"
#include "third_party/frc971/control_loops/state_feedback_loop.h"

#include "third_party/aos/common/time.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {
namespace testing {

using ::aos::time::Time;

using ::third_party::frc971::control_loops::drivetrain::y2016::
    MakeDrivetrainPlant;

using ::y2016::control_loops::drivetrain::GetDrivetrainConfig;

class DrivetrainPlant : public StateFeedbackPlant<4, 2, 2> {
 public:
  explicit DrivetrainPlant(StateFeedbackPlant<4, 2, 2>&& other)
      : StateFeedbackPlant<4, 2, 2>(::std::move(other)) {}

  void CheckU(const Eigen::Matrix<double, 2, 1> &U) override {
    EXPECT_LE(U(0, 0), U_max(0, 0) + 0.00001 + left_voltage_offset_);
    EXPECT_GE(U(0, 0), U_min(0, 0) - 0.00001 + left_voltage_offset_);
    EXPECT_LE(U(1, 0), U_max(1, 0) + 0.00001 + right_voltage_offset_);
    EXPECT_GE(U(1, 0), U_min(1, 0) - 0.00001 + right_voltage_offset_);
  }


  double left_voltage_offset() const { return left_voltage_offset_; }
  void set_left_voltage_offset(double left_voltage_offset) {
    left_voltage_offset_ = left_voltage_offset;
  }

  double right_voltage_offset() const { return right_voltage_offset_; }
  void set_right_voltage_offset(double right_voltage_offset) {
    right_voltage_offset_ = right_voltage_offset;
  }

 private:
  double left_voltage_offset_ = 0.0;
  double right_voltage_offset_ = 0.0;
};

// Class which simulates the drivetrain and sends out queue messages containing
// the position.
class DrivetrainSimulation {
 public:
  // Constructs a motor simulation.
  // TODO(aschuh) Do we want to test the clutch one too?
  DrivetrainSimulation(
      ::frc971::control_loops::drivetrain::InputQueue* input_queue,
      ::frc971::control_loops::drivetrain::OutputQueue* output_queue,
      ::muan::wpilib::gyro::GyroQueue* gyro_queue)
      : drivetrain_plant_(new DrivetrainPlant(MakeDrivetrainPlant())),
        input_queue_(input_queue),
        output_queue_(output_queue->MakeReader()),
        gyro_queue_(gyro_queue) {
    Reinitialize();
    last_U_.setZero();
  }

  // Resets the plant.
  void Reinitialize() {
    drivetrain_plant_->mutable_X(0, 0) = 0.0;
    drivetrain_plant_->mutable_X(1, 0) = 0.0;
    drivetrain_plant_->mutable_Y() =
        drivetrain_plant_->C() * drivetrain_plant_->X();
    last_left_position_ = drivetrain_plant_->Y(0, 0);
    last_right_position_ = drivetrain_plant_->Y(1, 0);
  }

  // Returns the position of the drivetrain.
  double GetLeftPosition() const { return drivetrain_plant_->Y(0, 0); }
  double GetRightPosition() const { return drivetrain_plant_->Y(1, 0); }

  // Sends out the position queue messages.
  void SendPositionMessage() {
    const double left_encoder = GetLeftPosition();
    const double right_encoder = GetRightPosition();

    {
      ::frc971::control_loops::drivetrain::InputProto position;
      position->set_left_encoder(left_encoder);
      position->set_right_encoder(right_encoder);
      input_queue_->WriteMessage(position);
    }

    {
      ::muan::wpilib::gyro::GyroMessageProto gyro;
      gyro->set_current_angle((right_encoder - left_encoder) /
                              (::third_party::frc971::control_loops::
                                   drivetrain::y2016::kRobotRadius *
                               2.0));
      gyro->set_current_angular_velocity(
          (drivetrain_plant_->X(3, 0) - drivetrain_plant_->X(1, 0)) /
          (::third_party::frc971::control_loops::drivetrain::y2016::
               kRobotRadius *
           2.0));
      gyro_queue_->WriteMessage(gyro);
    }
  }

  // Simulates the drivetrain moving for one timestep.
  void Simulate() {
    last_left_position_ = drivetrain_plant_->Y(0, 0);
    last_right_position_ = drivetrain_plant_->Y(1, 0);
    auto output = output_queue_.ReadLastMessage();
    ::Eigen::Matrix<double, 2, 1> U = last_U_;
    if (output) {
      last_U_ << (*output)->left_voltage(), (*output)->right_voltage();
      left_gear_high_ = (*output)->high_gear();
      right_gear_high_ = (*output)->high_gear();
    }
    {
      const double scalar = 1.0;  // ::aos::robot_state->voltage_battery / 12.0;
      last_U_ *= scalar;
    }

    if (left_gear_high_) {
      if (right_gear_high_) {
        drivetrain_plant_->set_index(3);
      } else {
        drivetrain_plant_->set_index(2);
      }
    } else {
      if (right_gear_high_) {
        drivetrain_plant_->set_index(1);
      } else {
        drivetrain_plant_->set_index(0);
      }
    }

    U(0, 0) += drivetrain_plant_->left_voltage_offset();
    U(1, 0) += drivetrain_plant_->right_voltage_offset();
    drivetrain_plant_->Update(U);
  }

  void set_left_voltage_offset(double left_voltage_offset) {
    drivetrain_plant_->set_left_voltage_offset(left_voltage_offset);
  }
  void set_right_voltage_offset(double right_voltage_offset) {
    drivetrain_plant_->set_right_voltage_offset(right_voltage_offset);
  }

  ::std::unique_ptr<DrivetrainPlant> drivetrain_plant_;

 private:
  ::frc971::control_loops::drivetrain::InputQueue* input_queue_;
  ::frc971::control_loops::drivetrain::OutputQueue::QueueReader output_queue_;

  ::muan::wpilib::gyro::GyroQueue* gyro_queue_;

  double last_left_position_;
  double last_right_position_;

  Eigen::Matrix<double, 2, 1> last_U_;

  bool left_gear_high_ = false;
  bool right_gear_high_ = false;
};

class DrivetrainTest : public ::testing::Test {
 protected:
  // Create a new instance of the test queue so that it invalidates the queue
  // that it points to.  Otherwise, we will have a pointer to shared memory that
  // is no longer valid.
  ::frc971::control_loops::drivetrain::GoalQueue goal_queue_;
  ::frc971::control_loops::drivetrain::InputQueue input_queue_;
  ::frc971::control_loops::drivetrain::StatusQueue status_queue_;
  ::frc971::control_loops::drivetrain::OutputQueue output_queue_;

  ::muan::wpilib::gyro::GyroQueue gyro_queue_;
  ::muan::wpilib::DriverStationQueue driver_station_queue_;

  // Create a loop and simulation plant.
  DrivetrainLoop drivetrain_motor_;
  DrivetrainSimulation drivetrain_motor_plant_;

  ::aos::monotonic_clock::time_point current_time_ = ::aos::monotonic_clock::epoch();

  DrivetrainTest()
      : drivetrain_motor_(GetDrivetrainConfig(), &goal_queue_, &input_queue_,
                          &output_queue_, &status_queue_,
                          &driver_station_queue_, &gyro_queue_),
        drivetrain_motor_plant_(&input_queue_, &output_queue_, &gyro_queue_) {}

  void SetUp() override {
    current_time_ = ::aos::monotonic_clock::epoch();
    ::aos::time::EnableMockTime(current_time_);
    ::aos::time::SetMockTime(current_time_);
  }

  void RunIteration() {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Update();
    drivetrain_motor_plant_.Simulate();
    SimulateTimestep(true);
  }

  void RunForTime(const aos::monotonic_clock::duration run_for) {
    ::aos::time::SetMockTime(current_time_);
    const auto end_time = aos::monotonic_clock::now() + run_for;
    while (::aos::monotonic_clock::now() < end_time) {
      RunIteration();
    }
  }

  void SimulateTimestep(bool enabled) {
    {
      ::muan::wpilib::DriverStationProto ds_state;
      ds_state->set_mode(enabled ? RobotMode::TELEOP : RobotMode::DISABLED);
      ds_state->set_is_sys_active(enabled);
      ds_state->set_brownout(false);
      ds_state->set_battery_voltage(12.0);
      ds_state->set_has_ds_connection(true);
      driver_station_queue_.WriteMessage(ds_state);
    }
    constexpr auto kTimeTick = ::std::chrono::milliseconds(5);
    ::aos::time::SetMockTime(current_time_ += kTimeTick);
  }

  void VerifyNearGoal() {
    auto goal = goal_queue_.MakeReader().ReadLastMessage();
    auto position = input_queue_.MakeReader().ReadLastMessage();

    EXPECT_TRUE(goal);
    EXPECT_TRUE((*goal)->has_distance_command());
    EXPECT_TRUE(position);

    EXPECT_NEAR((*goal)->distance_command().left_goal(),
                drivetrain_motor_plant_.GetLeftPosition(), 1e-3);
    EXPECT_NEAR((*goal)->distance_command().right_goal(),
                drivetrain_motor_plant_.GetRightPosition(), 1e-3);
  }

  virtual ~DrivetrainTest() {}
};

// Tests that the drivetrain converges on a goal.
TEST_F(DrivetrainTest, ConvergesCorrectly) {
  ::frc971::control_loops::drivetrain::GoalProto goal;
  goal->mutable_distance_command()->set_left_goal(-1.0);
  goal->mutable_distance_command()->set_right_goal(1.0);

  goal_queue_.WriteMessage(goal);

  RunForTime(::std::chrono::seconds(2));

  VerifyNearGoal();
}

// Tests that the drivetrain converges on a goal when under the effect of a
// voltage offset/disturbance.
TEST_F(DrivetrainTest, ConvergesWithVoltageError) {
  ::frc971::control_loops::drivetrain::GoalProto goal;
  goal->mutable_distance_command()->set_left_goal(-1.0);
  goal->mutable_distance_command()->set_right_goal(1.0);

  goal_queue_.WriteMessage(goal);

  drivetrain_motor_plant_.set_left_voltage_offset(1.0);
  drivetrain_motor_plant_.set_right_voltage_offset(1.0);
  RunForTime(::std::chrono::milliseconds(1500));
  VerifyNearGoal();
}

// Tests that it survives disabling.
TEST_F(DrivetrainTest, SurvivesDisabling) {
  ::frc971::control_loops::drivetrain::GoalProto goal;
  goal->mutable_distance_command()->set_left_goal(-1.0);
  goal->mutable_distance_command()->set_right_goal(1.0);

  goal_queue_.WriteMessage(goal);

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
      auto output = output_queue_.MakeReader().ReadLastMessage();
      ASSERT_TRUE(output);
      EXPECT_NEAR((*output)->left_voltage(), 0, 1e-7);
      EXPECT_NEAR((*output)->right_voltage(), 0, 1e-7);
    }
  }
  VerifyNearGoal();
}

// Tests that never having a goal doesn't break.
TEST_F(DrivetrainTest, NoGoalStart) {
  for (int i = 0; i < 20; ++i) {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Update();
    drivetrain_motor_plant_.Simulate();
  }
}

// Tests that never having a goal, but having driver's station messages, doesn't
// break.
TEST_F(DrivetrainTest, NoGoalWithRobotState) {
  RunForTime(::std::chrono::milliseconds(100));
}

// Tests that the robot successfully drives straight forward.
// This used to not work due to a U-capping bug.
TEST_F(DrivetrainTest, DriveStraightForward) {
  ::frc971::control_loops::drivetrain::GoalProto goal;
  goal->mutable_distance_command()->set_left_goal(4.0);
  goal->mutable_distance_command()->set_right_goal(4.0);

  goal_queue_.WriteMessage(goal);

  for (int i = 0; i < 600; ++i) {
    RunIteration();
    auto output = output_queue_.MakeReader().ReadLastMessage();
    ASSERT_TRUE(output);
    EXPECT_NEAR((*output)->left_voltage(), (*output)->right_voltage(), 1e-3);
    EXPECT_GE((*output)->left_voltage(), -12.001);
    EXPECT_GE((*output)->right_voltage(), -12.001);
  }
  VerifyNearGoal();
}

// Tests that the robot successfully drives close to straight.
// This used to fail in simulation due to libcdd issues with U-capping.
TEST_F(DrivetrainTest, DriveAlmostStraightForward) {
  ::frc971::control_loops::drivetrain::GoalProto goal;
  goal->mutable_distance_command()->set_left_goal(4.0);
  goal->mutable_distance_command()->set_right_goal(3.9);

  goal_queue_.WriteMessage(goal);

  for (int i = 0; i < 600; ++i) {
    RunIteration();
    auto output = output_queue_.MakeReader().ReadLastMessage();
    ASSERT_TRUE(output);
    EXPECT_GE((*output)->left_voltage(), -12.001);
    EXPECT_GE((*output)->right_voltage(), -12.001);
  }
  VerifyNearGoal();
}

// Tests that converting from a left, right position to a distance, angle
// coordinate system and back returns the same answer.
TEST_F(DrivetrainTest, LinearToAngularAndBack) {
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
TEST_F(DrivetrainTest, ProfileStraightForward) {
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
    goal_queue_.WriteMessage(goal);
  }

  while (aos::time::Time::Now() < aos::time::Time::InSeconds(6)) {
    RunIteration();

    auto output = output_queue_.MakeReader().ReadLastMessage();
    ASSERT_TRUE(output);
    EXPECT_GE((*output)->left_voltage(), -12.001);
    EXPECT_GE((*output)->right_voltage(), -12.001);
    EXPECT_NEAR((*output)->left_voltage(), (*output)->right_voltage(), 1e-3);
    EXPECT_GE((*output)->left_voltage(), -12.001);
    EXPECT_GE((*output)->right_voltage(), -12.001);
    EXPECT_LE((*output)->left_voltage(), 12.001);
    EXPECT_LE((*output)->right_voltage(), 12.001);
  }
  VerifyNearGoal();
}

// Tests that an angular motion profile succeeds.
TEST_F(DrivetrainTest, ProfileTurn) {
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
    goal_queue_.WriteMessage(goal);
  }

  while (aos::time::Time::Now() < aos::time::Time::InSeconds(6)) {
    RunIteration();

    auto output = output_queue_.MakeReader().ReadLastMessage();
    ASSERT_TRUE(output);
    EXPECT_GE((*output)->left_voltage(), -12.001);
    EXPECT_GE((*output)->right_voltage(), -12.001);
    EXPECT_NEAR((*output)->left_voltage(), -(*output)->right_voltage(), 1e-2);
    EXPECT_GE((*output)->left_voltage(), -12.001);
    EXPECT_GE((*output)->right_voltage(), -12.001);
    EXPECT_LE((*output)->left_voltage(), 12.001);
    EXPECT_LE((*output)->right_voltage(), 12.001);
  }
  VerifyNearGoal();
}

// Tests that a mixed turn drive saturated profile succeeds.
TEST_F(DrivetrainTest, SaturatedTurnDrive) {
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
    goal_queue_.WriteMessage(goal);
  }

  while (aos::time::Time::Now() < aos::time::Time::InSeconds(3.5)) {
    RunIteration();
    ASSERT_TRUE(output_queue_.MakeReader().ReadLastMessage());
  }
  VerifyNearGoal();
}

// Tests that being in teleop drive for a bit and then transitioning to closed
// drive profiles nicely.
TEST_F(DrivetrainTest, OpenLoopThenClosed) {
  {
    ::frc971::control_loops::drivetrain::GoalProto goal;
    goal->mutable_teleop_command()->set_closed_loop(true);
    goal->mutable_teleop_command()->set_steering(0.0);
    goal->mutable_teleop_command()->set_throttle(1.0);
    goal->set_gear(Gear::kHighGear);
    goal->mutable_teleop_command()->set_quick_turn(false);
    goal_queue_.WriteMessage(goal);
  }

  RunForTime(::std::chrono::milliseconds(1000));

  {
    ::frc971::control_loops::drivetrain::GoalProto goal;
    goal->mutable_teleop_command()->set_closed_loop(true);
    goal->mutable_teleop_command()->set_steering(0.0);
    goal->mutable_teleop_command()->set_throttle(-0.3);
    goal->set_gear(Gear::kHighGear);
    goal->mutable_teleop_command()->set_quick_turn(false);
    goal_queue_.WriteMessage(goal);
  }

  RunForTime(::std::chrono::milliseconds(1000));

  {
    ::frc971::control_loops::drivetrain::GoalProto goal;
    goal->mutable_teleop_command()->set_closed_loop(true);
    goal->mutable_teleop_command()->set_steering(0.0);
    goal->mutable_teleop_command()->set_throttle(0.0);
    goal->set_gear(Gear::kHighGear);
    goal->mutable_teleop_command()->set_quick_turn(false);
    goal_queue_.WriteMessage(goal);
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
    goal_queue_.WriteMessage(goal);
  }

  const auto end_time = aos::time::Time::Now() + aos::time::Time::InSeconds(4);
  while (aos::time::Time::Now() < end_time) {
    drivetrain_motor_plant_.SendPositionMessage();
    drivetrain_motor_.Update();
    drivetrain_motor_plant_.Simulate();
    SimulateTimestep(true);

    auto output = output_queue_.MakeReader().ReadLastMessage();
    ASSERT_TRUE(output);
    EXPECT_GE((*output)->left_voltage(), -12.001);
    EXPECT_GE((*output)->right_voltage(), -12.001);
    EXPECT_LE((*output)->left_voltage(), 12.001);
    EXPECT_LE((*output)->right_voltage(), 12.001);
  }
  VerifyNearGoal();
}


TEST_F(DrivetrainTest, CartesianEstimation) {
  ::frc971::control_loops::drivetrain::GoalProto goal;
  goal->mutable_distance_command()->set_left_goal(2.0);
  goal->mutable_distance_command()->set_right_goal(2.0);

  goal_queue_.WriteMessage(goal);

  RunForTime(::std::chrono::seconds(6));

  double r = ::third_party::frc971::control_loops::drivetrain::y2016::kRobotRadius;
  goal->mutable_distance_command()->set_left_goal(2.0 - r * M_PI / 2.0);
  goal->mutable_distance_command()->set_right_goal(2.0 + r * M_PI / 2.0);

  goal_queue_.WriteMessage(goal);

  RunForTime(::std::chrono::seconds(6));
  VerifyNearGoal();

  goal->mutable_distance_command()->set_left_goal(2.0 - r * M_PI / 2.0 + 1.0);
  goal->mutable_distance_command()->set_right_goal(2.0 + r * M_PI / 2.0 + 1.0);

  goal_queue_.WriteMessage(goal);

  RunForTime(::std::chrono::seconds(5));

  {
    auto maybe_status = status_queue_.ReadLastMessage();
    ASSERT_TRUE(maybe_status);
    ::frc971::control_loops::drivetrain::StatusProto status = *maybe_status;
    EXPECT_NEAR(status->estimated_x_position(), 2.0, 1e-3);
    EXPECT_NEAR(status->estimated_y_position(), 1.0, 1e-3);
  }
}

TEST_F(DrivetrainTest, PathDrive) {
  ::frc971::control_loops::drivetrain::GoalProto goal;
  goal->mutable_path_command()->set_x_goal(1.0);
  goal->mutable_path_command()->set_y_goal(1.0);
  goal->mutable_path_command()->set_theta_goal(0.0);

  goal->mutable_linear_constraints()->set_max_velocity(1.0);
  goal->mutable_linear_constraints()->set_max_acceleration(1.0);
  goal->mutable_angular_constraints()->set_max_velocity(1.0);
  goal->mutable_angular_constraints()->set_max_acceleration(1.0);

  goal_queue_.WriteMessage(goal);
  RunForTime(::std::chrono::seconds(7));
  {
    auto maybe_status = status_queue_.ReadLastMessage();
    ASSERT_TRUE(maybe_status);
    ::frc971::control_loops::drivetrain::StatusProto status = *maybe_status;
    EXPECT_NEAR(status->estimated_x_position(), 1.0, 1e-1);
    EXPECT_NEAR(status->estimated_y_position(), 1.0, 1e-1);
    EXPECT_NEAR(status->estimated_heading(), 0.0, 1e-2);
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
