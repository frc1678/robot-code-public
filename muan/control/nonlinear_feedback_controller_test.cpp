#include "muan/control/nonlinear_feedback_controller.h"
#include "gtest/gtest.h"
#include "muan/control/trajectory.h"

namespace muan {
namespace control {

DrivetrainModel GenerateModel() {
  DrivetrainModel::Properties properties;
  {
    properties.mass = 60.0;
    properties.angular_drag = 12.0;
    properties.wheel_radius = 0.0508;
    properties.wheelbase_radius = 0.36;
    properties.moment_inertia = properties.mass * properties.wheelbase_radius *
                                properties.wheelbase_radius;
  }

  DriveTransmission::Properties trans_properties;
  {
    trans_properties.speed_per_volt = 1 / 0.14;
    trans_properties.torque_per_volt =
        (properties.wheel_radius * properties.wheel_radius * properties.mass) /
        (2.0 * 0.012);
    trans_properties.friction_voltage = 1.01;
  }

  return DrivetrainModel(properties, DriveTransmission(trans_properties),
                         DriveTransmission(trans_properties));
}

class TestFixture : public ::testing::Test {
 public:
  void Run(Pose initial, Pose final, double initial_velocity = 0,
           double final_velocity = 0, bool backwards = false,
           double extra_distance_initial = 0, double extra_distance_final = 0,
           double initial_angular_velocity = 0,
           double final_angular_velocity = 0, bool high_gear = true) {
    HermiteSpline spline(initial, final, initial_velocity, final_velocity,
                         backwards, extra_distance_initial,
                         extra_distance_final, initial_angular_velocity,
                         final_angular_velocity);

    Trajectory trajectory(spline, constraints_, high_gear, model_);

    Trajectory::TimedPose sample = trajectory.Advance(0.04);
    Pose current_pose = sample.pose.pose();
    NonLinearFeedbackController::Output goal;

    Pose error;

    double t = 0.0;
    while (!trajectory.done()) {
      t += 0.04;
      Trajectory::TimedPose prev_sample = sample;
      sample = trajectory.Advance(0.04);

      error = sample.pose.pose() - current_pose;

      double omega = sample.v * sample.pose.curvature();

      goal = controller_.Update(
          (Eigen::Vector2d() << sample.v, omega).finished(),
          (Eigen::Vector2d() << sample.a, sample.a * sample.pose.curvature())
              .finished(),
          current_pose, error, high_gear);

      Eigen::Vector2d velocity = model_.ForwardKinematics(goal.velocity) * 0.9;

      Eigen::Vector3d delta;
      delta(0) = velocity(0) * 0.04 *
                 std::cos(current_pose.heading() + velocity(1) * 0.04);
      delta(1) = velocity(0) * 0.04 *
                 std::sin(current_pose.heading() + velocity(1) * 0.04);
      delta(2) = velocity(1) * 0.04;

      current_pose = Pose(current_pose.Get() + delta);

      EXPECT_LT(std::abs(delta(0)), constraints_.max_velocity + 1e-9);
      EXPECT_LT(std::abs(goal.feedforwards(0)), 12.1);
      EXPECT_LT(std::abs(goal.feedforwards(1)), 12.1);

      Eigen::Vector2d estimated_response = model_.ForwardDynamics(
          (Eigen::Vector2d() << sample.v, omega).finished(), goal.feedforwards,
          high_gear);
      EXPECT_NEAR(sample.a, estimated_response(0), 1e-2);
      EXPECT_NEAR(sample.a * sample.pose.curvature(), estimated_response(1),
                  1e-2);
    }

    EXPECT_LT(error.translational().norm(), 3e-2);  // ~3cm within the goal
    EXPECT_LT(error.heading(), 3e-2);
  }

 private:
  DrivetrainModel model_ = GenerateModel();
  NonLinearFeedbackController controller_{GenerateModel(), 3.0, 1.5};
  Trajectory::Constraints constraints_{
      .max_velocity = 4.,
      .max_voltage = 12.,
      .max_acceleration = 3.,
      .max_centripetal_acceleration = 1.,

      .initial_velocity = 0.,
      .final_velocity = 0.,
  };
};

TEST_F(TestFixture, StraightLine) {
  Pose a((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Pose b((Eigen::Vector3d() << 3.0, 0.0, 0.0).finished());

  Run(a, b);
}

TEST_F(TestFixture, SimpleSCurve) {
  Pose a((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Pose b((Eigen::Vector3d() << 1.0, 1.0, 0.0).finished());
  Run(a, b);
}

TEST_F(TestFixture, Reversed) {
  Pose a((Eigen::Vector3d() << 1.0, 1.0, 0.0).finished());
  Pose b((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Run(a, b);
}

TEST_F(TestFixture, HeadingBackwards) {
  Pose a((Eigen::Vector3d() << 0.0, 0.0, M_PI / 2.0).finished());
  Pose b((Eigen::Vector3d() << -1.0, -1.0, -M_PI / 2.0).finished());
  Run(a, b);
}

TEST_F(TestFixture, Wraparound) {
  // This test crosses theta=+-pi twice, once in each direction
  Pose a((Eigen::Vector3d() << 0.0, 0.0, M_PI / 2).finished());
  Pose b((Eigen::Vector3d() << -1.0, 0.0, M_PI / 2).finished());
  Run(a, b);
}

TEST_F(TestFixture, DrivesBackwards) {
  Pose a((Eigen::Vector3d() << 1.0, 1.0, 0.0).finished());
  Pose b((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Run(a, b, 0, 0, true);
}

TEST_F(TestFixture, ExtraDistance) {
  Pose a((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Pose b((Eigen::Vector3d() << 2.0, 2.0, 0.0).finished());
  Run(a, b, 0, 0, false, 1.0, 0);
}

TEST_F(TestFixture, InitialVelocity) {
  Pose a((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Pose b((Eigen::Vector3d() << -6.65, -1.0, M_PI * -0.3).finished());
  Run(a, b, 0.3991, 0, true);
}

TEST_F(TestFixture, AngularVelocity) {
  Pose a((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Pose b((Eigen::Vector3d() << 3.0, 0.0, 0.0).finished());
  Run(a, b, 2, 1, false, 0.5, 0.5, 1, -1);
}

TEST_F(TestFixture, AngularVelocityReverse) {
  Pose a((Eigen::Vector3d() << 2.0, 1.0, 1.6).finished());
  Pose b((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Run(a, b, -1, -1, true, 0, 0, -1.6, -1.6);
}

TEST_F(TestFixture, ExtraTest) {
  Pose a((Eigen::Vector3d() << -6.40, -0.8, 0.5).finished());
  Pose b((Eigen::Vector3d() << -5.0, -1.2, -0.38).finished());
  Run(a, b, 0, 0, false);
}

}  // namespace control
}  // namespace muan
