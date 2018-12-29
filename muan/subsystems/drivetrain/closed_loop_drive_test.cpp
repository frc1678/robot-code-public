#include "muan/subsystems/drivetrain/closed_loop_drive.h"
#include "gtest/gtest.h"

namespace muan {
namespace subsystems {
namespace drivetrain {

constexpr double kStallTorque = 1.41;
constexpr double kStallCurrent = 89;
constexpr double kFreeSpeed = 5840 * 2 * M_PI / 60;
constexpr double kFreeCurrent = 3;

constexpr double kMass = 65.;
constexpr double kDistRadius = 0.45;
constexpr double kMoment = kMass * kDistRadius * kDistRadius;

constexpr double kForceStiction = 32.65;
constexpr double kForceFriction = 20;
constexpr double kAngularDrag = -12;

constexpr double kRobotRadius = 0.438;
constexpr double kWheelRadius = 6.0 * 0.0254 / 2.0;

constexpr double kHighGearRatio = (12.0 / 50.0) * (18.0 / 46.0) * (50.0 / 34.0);
constexpr double kLowGearRatio = (12.0 / 50.0) * (18.0 / 46.0) * (34.0 / 50.0);

constexpr double kHighGearEfficiency = 0.75;
constexpr double kLowGearEfficiency = 0.8;

muan::subsystems::drivetrain::DrivetrainConfig GetDrivetrainConfig() {
  muan::control::DriveTransmission::Properties high_gear{
      .num_motors = 3,
      .motor_kt = kStallTorque / kStallCurrent,
      .motor_kv = (12 - kFreeCurrent * (12 / kStallCurrent)) / kFreeSpeed,
      .motor_resistance = 12 / kStallCurrent,
      .gear_ratio = kHighGearRatio,
      .efficiency = kHighGearEfficiency,
  };

  muan::control::DriveTransmission::Properties low_gear{
      .num_motors = 3,
      .motor_kt = kStallTorque / kStallCurrent,
      .motor_kv = (12 - kFreeCurrent * (12 / kStallCurrent)) / kFreeSpeed,
      .motor_resistance = 12 / kStallCurrent,
      .gear_ratio = kLowGearRatio,
      .efficiency = kLowGearEfficiency,
  };

  muan::control::DrivetrainModel::Properties model{
      .wheelbase_radius = kRobotRadius,
      .angular_drag = kAngularDrag,  // TUNE ME
      .mass = kMass,
      .moment_inertia = kMoment,
      .force_stiction = kForceStiction,  // TUNE ME
      .force_friction = kForceFriction,  // TUNE ME
      .wheel_radius = kWheelRadius,
  };

  return {
      .high_gear_wheel_non_linearity = 0.65,
      .low_gear_wheel_non_linearity = 0.5,
      .high_gear_sensitivity = 0.95,
      .low_gear_sensitivity = 0.65,
      .beta = 2.0,
      .zeta = 0.7,
      .dt = 0.01,

      .max_velocity = 3.4,
      .max_acceleration = 6.0,
      .max_centripetal_acceleration = M_PI / 2.0,

      .high_gear_properties = high_gear,
      .low_gear_properties = low_gear,
      .drive_properties = model,
  };
}

class DrivetrainTest : public ::testing::Test {
 public:
  void RunFor(int ticks) {
    for (int i = 0; i < ticks; i++) {
      Update();
    }
  }

  void Update() {
    closed_loop_drive_.SetGoal(goal_);
    closed_loop_drive_.Update(&output_, &status_);
    SetInputs();
  }

  void SetInputs() {
    if (output_->output_type() == POSITION) {
      Eigen::Vector2d current = model_.ForwardKinematics(
          Eigen::Vector2d(output_->left_setpoint(), output_->right_setpoint()));
      integrated_heading_ = current(1);
      cartesian_position_ = Eigen::Vector2d(current(0) * std::cos(current(1)),
                                            current(0) * std::sin(current(1)));
      left_right_position_ =
          Eigen::Vector2d(output_->left_setpoint(), output_->right_setpoint());
    } else if (output_->output_type() == VELOCITY) {
      Eigen::Vector2d velocity = model_.ForwardKinematics(
          Eigen::Vector2d(output_->left_setpoint(), output_->right_setpoint()));
      integrated_heading_ += 0.01 * velocity(1);
      cartesian_position_(0) +=
          velocity(0) * 0.01 * std::cos(integrated_heading_);
      cartesian_position_(1) +=
          velocity(0) * 0.01 * std::sin(integrated_heading_);
      linear_angular_velocity_ = velocity;

      left_right_position_(0) += output_->left_setpoint() * 0.01;
      left_right_position_(1) += output_->right_setpoint() * 0.01;
    }
  }

  GoalProto goal_;
  OutputProto output_;
  StatusProto status_;

  double integrated_heading_;
  Eigen::Vector2d cartesian_position_;
  Eigen::Vector2d left_right_position_;
  Eigen::Vector2d linear_angular_velocity_;

  DrivetrainModel model_{
      (GetDrivetrainConfig()).drive_properties,
      DriveTransmission((GetDrivetrainConfig()).low_gear_properties),
      DriveTransmission((GetDrivetrainConfig()).high_gear_properties)};

 private:
  ClosedLoopDrive closed_loop_drive_{
      GetDrivetrainConfig(), &cartesian_position_, &integrated_heading_,
      &linear_angular_velocity_, &left_right_position_};
};

TEST_F(DrivetrainTest, PointTurn) {
  integrated_heading_ = 0.;
  left_right_position_ = Eigen::Vector2d(0, 0);
  cartesian_position_ = Eigen::Vector2d(0, 0);
  goal_->set_point_turn_goal(M_PI / 2.);
  Update();
  EXPECT_NEAR(integrated_heading_, M_PI / 2., 1e-6);
  EXPECT_NEAR(cartesian_position_.norm(), 0., 1e-6);
}

TEST_F(DrivetrainTest, Distance) {
  integrated_heading_ = 0.;
  left_right_position_ = Eigen::Vector2d(0, 0);
  cartesian_position_ = Eigen::Vector2d(0, 0);
  goal_->set_distance_goal(1.);
  Update();
  EXPECT_NEAR(integrated_heading_, 0., 1e-6);
  EXPECT_NEAR(cartesian_position_.norm(), 1., 1e-6);
}

TEST_F(DrivetrainTest, LeftRight) {
  integrated_heading_ = 0.;
  left_right_position_ = Eigen::Vector2d(0, 0);
  cartesian_position_ = Eigen::Vector2d(0, 0);
  Eigen::Vector2d delta =
      model_.InverseKinematics(Eigen::Vector2d(1., M_PI / 4.));
  goal_->mutable_left_right_goal()->set_left_goal(delta(0));
  goal_->mutable_left_right_goal()->set_right_goal(delta(1));
  Update();
  EXPECT_NEAR(integrated_heading_, M_PI / 4., 1e-6);
  EXPECT_NEAR(cartesian_position_.norm(), 1., 1e-6);
}

TEST_F(DrivetrainTest, PathFollow) {
  integrated_heading_ = 0.;
  left_right_position_ = Eigen::Vector2d(0, 0);
  cartesian_position_ = Eigen::Vector2d(0, 0);
  linear_angular_velocity_ = Eigen::Vector2d(0, 0);
  goal_->mutable_path_goal()->set_x(1.);
  goal_->mutable_path_goal()->set_y(1.);
  goal_->mutable_path_goal()->set_heading(0.);
  goal_->mutable_path_goal()->set_max_voltage(12.);
  goal_->mutable_path_goal()->set_backwards(false);
  RunFor(500);
  EXPECT_NEAR(cartesian_position_(0), 1, 1e-2);
  EXPECT_NEAR(cartesian_position_(1), 1, 1e-2);
  EXPECT_NEAR(integrated_heading_, 0, 1e-2);
}

TEST_F(DrivetrainTest, PathFollowBackwards) {
  integrated_heading_ = M_PI;
  left_right_position_ = Eigen::Vector2d(0, 0);
  cartesian_position_ = Eigen::Vector2d(0, 0);
  linear_angular_velocity_ = Eigen::Vector2d(0, 0);
  goal_->mutable_path_goal()->set_x(1.);
  goal_->mutable_path_goal()->set_y(1.);
  goal_->mutable_path_goal()->set_heading(M_PI);
  goal_->mutable_path_goal()->set_max_voltage(12.);
  goal_->mutable_path_goal()->set_backwards(true);
  RunFor(500);
  EXPECT_NEAR(cartesian_position_(0), 1, 1e-2);
  EXPECT_NEAR(cartesian_position_(1), 1, 1e-2);
  EXPECT_NEAR(integrated_heading_, M_PI, 2e-2);
}

TEST_F(DrivetrainTest, Transition) {
  integrated_heading_ = 0.;
  left_right_position_ = Eigen::Vector2d(0, 0);
  cartesian_position_ = Eigen::Vector2d(0, 0);
  linear_angular_velocity_ = Eigen::Vector2d(0, 0);

  goal_->set_point_turn_goal(M_PI / 3.);
  Update();
  EXPECT_NEAR(integrated_heading_, M_PI / 3., 1e-2);
  EXPECT_NEAR(cartesian_position_(0), 0, 1e-2);
  EXPECT_NEAR(cartesian_position_(1), 0, 1e-2);

  Eigen::Vector2d prev_pos = cartesian_position_;
  double prev_head = integrated_heading_;
  goal_->set_distance_goal(0.5);
  Update();
  EXPECT_NEAR(integrated_heading_ - prev_head, 0., 1e-6);
  EXPECT_NEAR((cartesian_position_ - prev_pos).norm(), 0.5, 1e-6);

  prev_pos = cartesian_position_;
  prev_head = integrated_heading_;
  Eigen::Vector2d delta =
      model_.InverseKinematics(Eigen::Vector2d(0., M_PI / 4.));
  goal_->mutable_left_right_goal()->set_left_goal(left_right_position_(0) +
                                                  delta(0));
  goal_->mutable_left_right_goal()->set_right_goal(left_right_position_(1) +
                                                   delta(1));
  Update();
  EXPECT_NEAR(integrated_heading_ - prev_head, M_PI / 4., 1e-6);

  prev_pos = cartesian_position_;
  prev_head = integrated_heading_;
  delta = model_.InverseKinematics(Eigen::Vector2d(1., 0.));
  goal_->mutable_left_right_goal()->set_left_goal(left_right_position_(0) +
                                                  delta(0));
  goal_->mutable_left_right_goal()->set_right_goal(left_right_position_(1) +
                                                   delta(1));
  Update();
  EXPECT_NEAR((cartesian_position_ - prev_pos).norm(), 1., 1e-6);
}

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace muan
