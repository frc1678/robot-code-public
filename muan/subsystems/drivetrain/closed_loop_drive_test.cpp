#include "muan/subsystems/drivetrain/closed_loop_drive.h"
/* #include <fstream> */
#include "gtest/gtest.h"

namespace muan {
namespace subsystems {
namespace drivetrain {

constexpr double kRobotRadius = 0.3489513;
constexpr double kWheelRadius = 4.0 * 0.0254 / 2.0;

constexpr double kMaxVoltage = 12.0;
constexpr double kFreeSpeed = 4.34 / kWheelRadius;  // rad / s

constexpr double kDriveKv = kMaxVoltage / kFreeSpeed;  // V / (rad / s)
constexpr double kDriveKa = 0.012;                     // V / (rad / s^2)
constexpr double kDriveKs = 1.3;                       // V

constexpr double kMass = 60.;                                  // kg
constexpr double kDistRadius = 0.4;                            // m
constexpr double kMoment = kMass * kDistRadius * kDistRadius;  // kg * m^2

constexpr double kAngularDrag = 12.0;  // N*m / (rad / s)

muan::subsystems::drivetrain::DrivetrainConfig GetDrivetrainConfig() {
  muan::control::DriveTransmission::Properties high_gear{
      .speed_per_volt = 1.0 / kDriveKv,
      .torque_per_volt = kWheelRadius * kWheelRadius * kMass / (2.0 * kDriveKa),
      .friction_voltage = kDriveKs,
  };

  muan::control::DriveTransmission::Properties low_gear =
      high_gear;  // One-speed :D

  muan::control::DrivetrainModel::Properties model{
      .mass = kMass,
      .moment_inertia = kMoment,
      .angular_drag = kAngularDrag,  // TUNE ME
      .wheel_radius = kWheelRadius,
      .wheelbase_radius = kRobotRadius,
  };

  return {
      .high_gear_wheel_non_linearity = 0.65,
      .low_gear_wheel_non_linearity = 0.5,
      .high_gear_sensitivity = 0.65,
      .low_gear_sensitivity = 0.65,
      .beta = 2.0,
      .zeta = 0.7,
      .dt = 0.01,

      .max_velocity = 4.0,
      .max_acceleration = 3.0,
      .max_centripetal_acceleration = 1.0,

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
    uint64_t prev_timestamp = timestamp_;
    timestamp_ =
        std::chrono::duration_cast<std::chrono::milliseconds>(
            aos::monotonic_clock::now() - aos::monotonic_clock::epoch())
            .count();
    double dt = (timestamp_ - prev_timestamp) / 1000.;

    status_->set_dt(dt);

    closed_loop_drive_.SetGoal(goal_);
    closed_loop_drive_.Update(&output_, &status_);
    SetInputs();
    aos::time::IncrementMockTime(std::chrono::milliseconds(20));
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
      integrated_heading_ += status_->dt() * velocity(1);
      cartesian_position_(0) +=
          velocity(0) * status_->dt() * std::cos(integrated_heading_);
      cartesian_position_(1) +=
          velocity(0) * status_->dt() * std::sin(integrated_heading_);
      linear_angular_velocity_ = velocity;

      left_right_position_(0) += output_->left_setpoint() * status_->dt();
      left_right_position_(1) += output_->right_setpoint() * status_->dt();

      /* if (!status_->profile_complete()) { */
      /*   Eigen::Vector2d estimated_response = model_.ForwardDynamics( */
      /*       linear_angular_velocity_, */
      /*       Eigen::Vector2d(output_->left_setpoint_ff(), */
      /*                       output_->right_setpoint_ff()), */
      /*       output_->high_gear()); */
      /*   EXPECT_NEAR(status_->profiled_acceleration_goal(), */
      /*               estimated_response(0), 1e-1); */
      /*   file_ << status_->profiled_acceleration_goal() << "," */
      /*         << estimated_response(0) << "\n"; */
      /* } */
    }
  }

  GoalProto goal_;
  OutputProto output_;
  StatusProto status_;

  double integrated_heading_;
  Eigen::Vector2d cartesian_position_;
  Eigen::Vector2d left_right_position_;
  Eigen::Vector2d linear_angular_velocity_;

  uint64_t timestamp_;

  /* std::ofstream file_{"/tmp/test.csv"}; */

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
  aos::time::EnableMockTime(aos::monotonic_clock::epoch());
  integrated_heading_ = 0.;
  left_right_position_ = Eigen::Vector2d(0, 0);
  cartesian_position_ = Eigen::Vector2d(0, 0);
  goal_->set_point_turn_goal(M_PI / 2.);
  Update();
  Update();
  EXPECT_NEAR(integrated_heading_, M_PI / 2., 1e-6);
  EXPECT_NEAR(cartesian_position_.norm(), 0., 1e-6);
  EXPECT_TRUE(status_->point_turn_complete());
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
  EXPECT_NEAR(integrated_heading_, 0, 2e-2);
}

/* TEST_F(DrivetrainTest, Transition) { */
/*   integrated_heading_ = 0.; */
/*   left_right_position_ = Eigen::Vector2d(0, 0); */
/*   cartesian_position_ = Eigen::Vector2d(0, 0); */
/*   linear_angular_velocity_ = Eigen::Vector2d(0, 0); */

/*   goal_->set_point_turn_goal(M_PI / 3.); */
/*   Update(); */
/*   EXPECT_NEAR(integrated_heading_, M_PI / 3., 1e-2); */
/*   EXPECT_NEAR(cartesian_position_(0), 0, 1e-2); */
/*   EXPECT_NEAR(cartesian_position_(1), 0, 1e-2); */

/*   Eigen::Vector2d prev_pos = cartesian_position_; */
/*   double prev_head = integrated_heading_; */
/*   goal_->set_distance_goal(0.5); */
/*   Update(); */
/*   EXPECT_NEAR(integrated_heading_ - prev_head, 0., 1e-6); */
/*   EXPECT_NEAR((cartesian_position_ - prev_pos).norm(), 0.5, 1e-6); */

/*   prev_pos = cartesian_position_; */
/*   prev_head = integrated_heading_; */
/*   Eigen::Vector2d delta = */
/*       model_.InverseKinematics(Eigen::Vector2d(0., M_PI / 4.)); */
/*   goal_->mutable_left_right_goal()->set_left_goal(left_right_position_(0) +
 */

/*                                                   delta(0)); */
/*   goal_->mutable_left_right_goal()->set_right_goal(left_right_position_(1) +
 */

/*                                                    delta(1)); */
/*   Update(); */
/*   EXPECT_NEAR(integrated_heading_ - prev_head, M_PI / 4., 1e-6); */

/*   prev_pos = cartesian_position_; */
/*   prev_head = integrated_heading_; */
/*   delta = model_.InverseKinematics(Eigen::Vector2d(1., 0.)); */
/*   goal_->mutable_left_right_goal()->set_left_goal(left_right_position_(0) +
 */

/*                                                   delta(0)); */
/*   goal_->mutable_left_right_goal()->set_right_goal(left_right_position_(1) +
 */

/*                                                    delta(1)); */
/*   Update(); */
/*   EXPECT_NEAR((cartesian_position_ - prev_pos).norm(), 1., 1e-6); */
/* } */

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace muan
