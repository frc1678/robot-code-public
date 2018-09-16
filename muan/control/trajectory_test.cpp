#include "muan/control/trajectory.h"
#include "gtest/gtest.h"

namespace muan {
namespace control {

DrivetrainModel GenerateModel() {
  DrivetrainModel::Properties properties;
  properties.wheelbase_radius = 0.42;
  properties.angular_drag = -35;
  properties.mass = 50.0;
  properties.moment_inertia = 0.5 * properties.mass *
                              properties.wheelbase_radius *
                              properties.wheelbase_radius;
  properties.force_stiction = 20.0;
  properties.force_friction = 5.0;
  properties.wheel_radius = 3.25 / 2 * 0.0254;

  DriveTransmission::Properties trans_properties;
  const double i_stall = 131;
  const double t_stall = 2.41;
  const double i_free = 0;
  const double w_free = 5330 * (2 * M_PI / 60.0);
  {
    trans_properties.motor_kt = t_stall / i_stall;
    trans_properties.motor_resistance = 12.0 / i_stall;
    trans_properties.motor_kv =
        (12.0 - i_free * trans_properties.motor_resistance) / w_free;
    trans_properties.gear_ratio = 1 / 4.16;
    trans_properties.num_motors = 2;
  }

  return DrivetrainModel(properties, DriveTransmission(trans_properties),
                         DriveTransmission(trans_properties));
}

class TrajectoryTest : public ::testing::Test {
 public:
  TrajectoryTest() {}

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

    Trajectory::TimedPose sample = trajectory.SampleTime(0);

    EXPECT_NEAR(sample.pose.pose().Get()(0), initial.Get()(0), 1e-9);
    EXPECT_NEAR(sample.pose.pose().Get()(1), initial.Get()(1), 1e-9);
    EXPECT_NEAR(sample.pose.pose().Get()(2), initial.Get()(2), 1e-9);

    while (!trajectory.done()) {
      auto prev_sample = sample;
      sample = trajectory.Advance(0.01);

      EXPECT_LT(std::abs(sample.v), constraints_.max_velocity + 1e-9);
      EXPECT_LT(std::abs(sample.a), constraints_.max_acceleration + 1e-9);

      EXPECT_NEAR(FromMagDirection(prev_sample.v * (sample.t - prev_sample.t),
                                   prev_sample.pose.heading())(0) +
                      prev_sample.pose.translational()(0),
                  sample.pose.translational()(0), 1e-2);
      EXPECT_NEAR(FromMagDirection(prev_sample.v * (sample.t - prev_sample.t),
                                   prev_sample.pose.heading())(1) +
                      prev_sample.pose.translational()(1),
                  sample.pose.translational()(1), 1e-2);

      if (backwards) {
        EXPECT_LT(sample.v, 1e-9);
      } else {
        EXPECT_GT(sample.v, -1e-9);
      }
    }

    EXPECT_NEAR(sample.pose.pose().Get()(0), final.Get()(0), 1e-9);
    EXPECT_NEAR(sample.pose.pose().Get()(1), final.Get()(1), 1e-9);
    EXPECT_NEAR(sample.pose.pose().Get()(2), final.Get()(2), 1e-9);
  }

 private:
  Trajectory::Constraints constraints_{
      .max_velocity = 1.,
      .max_voltage = 12.,
      .max_acceleration = 1.,
      .max_centripetal_acceleration = M_PI / 2,

      .initial_velocity = 0.,
      .final_velocity = 0.,
  };

  DrivetrainModel model_ = GenerateModel();
};

TEST_F(TrajectoryTest, StraightLine) {
  Pose a((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Pose b((Eigen::Vector3d() << 3.0, 0.0, 0.0).finished());
  Run(a, b);
}

TEST_F(TrajectoryTest, SimpleSCurve) {
  Pose a((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Pose b((Eigen::Vector3d() << 1.0, 1.0, 0.0).finished());
  Run(a, b);
}

TEST_F(TrajectoryTest, Reversed) {
  Pose a((Eigen::Vector3d() << 1.0, 1.0, 0.0).finished());
  Pose b((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Run(a, b);
}

TEST_F(TrajectoryTest, HeadingBackwards) {
  Pose a((Eigen::Vector3d() << 0.0, 0.0, M_PI / 2.0).finished());
  Pose b((Eigen::Vector3d() << -1.0, -1.0, -M_PI / 2.0).finished());
  Run(a, b);
}

TEST_F(TrajectoryTest, Wraparound) {
  // This test crosses theta=+-pi twice, once in each direction
  Pose a((Eigen::Vector3d() << 0.0, 0.0, M_PI / 2).finished());
  Pose b((Eigen::Vector3d() << -1.0, 0.0, M_PI / 2).finished());
  Run(a, b);
}

TEST_F(TrajectoryTest, DrivesBackwards) {
  Pose a((Eigen::Vector3d() << 1.0, 1.0, 0.0).finished());
  Pose b((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Run(a, b, 0, 0, true);
}

TEST_F(TrajectoryTest, ExtraDistance) {
  Pose a((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Pose b((Eigen::Vector3d() << 2.0, 2.0, 0.0).finished());
  Run(a, b, 0, 0, false, 1.0, 0);
}

TEST_F(TrajectoryTest, InitialVelocity) {
  Pose a((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Pose b((Eigen::Vector3d() << -6.65, -1.0, M_PI * -0.3).finished());
  Run(a, b, 0.3991, 0, true);
}

TEST_F(TrajectoryTest, AngularVelocity) {
  Pose a((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Pose b((Eigen::Vector3d() << 3.0, 0.0, 0.0).finished());
  Run(a, b, 2, 1, false, 0.5, 0.5, 1, -1);
}

TEST_F(TrajectoryTest, AngularVelocityReverse) {
  Pose a((Eigen::Vector3d() << 2.0, 1.0, 1.6).finished());
  Pose b((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Run(a, b, -1, -1, true, 0, 0, -1.6, -1.6);
}

TEST_F(TrajectoryTest, ExtraTest) {
  Pose a((Eigen::Vector3d() << -6.40, -0.8, 0.5).finished());
  Pose b((Eigen::Vector3d() << -5.0, -1.2, -0.38).finished());
  Run(a, b, 0, 0, false);
}

}  // namespace control
}  // namespace muan
