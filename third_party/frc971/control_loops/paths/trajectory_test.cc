#include "third_party/frc971/control_loops/paths/trajectory.h"
#include <fstream>

#include "gtest/gtest.h"

#define _USE_MATH_DEFINES
#include <cmath>

#define EXPECT_ANGLE_NEAR(val1, val2, abs_error) \
EXPECT_NEAR(wrap_angle((val1) - (val2) + M_PI), M_PI, abs_error) \
<< "val1 = " << (val1) << "\tval2 = " << (val2) << "\n"

namespace frc971 {
namespace control_loops {
namespace paths {
namespace testing {

double wrap_angle(double angle) {
  angle = ::std::fmod(angle, 2.0 * M_PI);
  return angle > 0 ? angle : angle + 2.0 * M_PI;
}

double average_angle(double angle1, double angle2) {
  double angle_min = ::std::min(angle1, angle2);
  double angle_max = ::std::max(angle1, angle2);
  if (angle_max - angle_min < angle_min + 2 * M_PI - angle_max) {
    return (angle_min + angle_max) * 0.5;
  } else {
    return (angle_min + angle_max) * 0.5 + M_PI;
  }
}

class TrajectoryTest : public ::testing::Test {
 public:
  constexpr static double kRadius = 0.59 / 2.0;
  constexpr static double kMaxAccel = 3.0;
  constexpr static double kMaxVoltage = 9.0;

  void SetUp() override {
    trajectory_.set_maximum_acceleration(kMaxAccel);
    trajectory_.set_maximum_voltage(kMaxVoltage);

    Eigen::Matrix<double, 4, 4> A = Eigen::Matrix<double, 4, 4>::Zero();
    A(0, 1) = 1.0;
    A(1, 1) = -2.1;
    A(1, 3) = 0.86;
    A(2, 3) = 1.0;
    A(3, 3) = -2.1;
    A(3, 1) = 0.86;

    Eigen::Matrix<double, 4, 2> B = Eigen::Matrix<double, 4, 2>::Zero();
    B(1, 0) = 1.22;
    B(1, 1) = -0.50;
    B(3, 1) = 1.22;
    B(3, 0) = -0.50;

    trajectory_.set_system(A, B, kRadius);
    trajectory_.Reset();
  }

  void Run(Pose initial, Pose final,
           double initial_velocity = 0, double final_velocity = 0,
           bool backwards = false,
           double extra_distance_initial = 0, double extra_distance_final = 0,
           double initial_angular_velocity = 0,
           double final_angular_velocity = 0) {
    path_ = HermitePath(initial, final, initial_velocity, final_velocity,
                        backwards, extra_distance_initial, extra_distance_final,
                        initial_angular_velocity, final_angular_velocity);

    initial_state_ = (State() <<
            initial.translational()(0) - initial.heading() * kRadius,
            initial_velocity - initial_angular_velocity * kRadius,
            initial.translational()(0) + initial.heading() * kRadius,
            initial_velocity + initial_angular_velocity * kRadius).finished();

    final_velocity_ = final_velocity;
    final_angular_velocity_ = final_angular_velocity;

    trajectory_.SetPath(path_, initial_state_,
                        final_velocity_, final_angular_velocity_);

    Trajectory::Sample s_last = {};
    s_last.pose = initial;
    s_last.drivetrain_state = initial_state_;

    for (size_t i = 0; i < 2000; i++) {
      Trajectory::Sample s = trajectory_.Update();
      if (s.profile_complete) {
        s_last = s;
        break;
      }

      // Continuity/smoothness
      EXPECT_NEAR(s_last.drivetrain_state(0), s.drivetrain_state(0), 0.05)
          << "at i=" << i;
      EXPECT_NEAR(s_last.drivetrain_state(1), s.drivetrain_state(1),
                  0.006 * kMaxAccel)
          << "at i=" << i;
      EXPECT_NEAR(s_last.drivetrain_state(2), s.drivetrain_state(2), 0.05)
          << " at i=" << i;
      EXPECT_NEAR(s_last.drivetrain_state(3), s.drivetrain_state(3),
                  0.006 * kMaxAccel)
          << "at i=" << i;
      EXPECT_NEAR(s_last.pose.Get()(0), s.pose.Get()(0), 0.05) << "at i=" << i;
      EXPECT_NEAR(s_last.pose.Get()(1), s.pose.Get()(1), 0.05) << "at i=" << i;
      EXPECT_ANGLE_NEAR(s_last.pose.Get()(2), s.pose.Get()(2), 0.05)
          << "at i=" << i;

      // Consistency of drivetrain delta position with drivetrain velocity
      /*EXPECT_NEAR((s.drivetrain_state(0) - s_last.drivetrain_state(0)) / 0.005,
                  (s.drivetrain_state(1) + s_last.drivetrain_state(1)) * 0.5,
                  0.0002 / 0.005)
          << "at i=" << i;
      EXPECT_NEAR((s.drivetrain_state(2) - s_last.drivetrain_state(2)) / 0.005,
                  (s.drivetrain_state(3) + s_last.drivetrain_state(3)) * 0.5,
                  0.0002 / 0.005)
          << "at i=" << i;*/

      // Consistency of pose delta position with pose heading
      bool current_backwards = s.drivetrain_state(0) + s.drivetrain_state(2) <
                               s_last.drivetrain_state(0) + s_last.drivetrain_state(2);
      double measured_angle = (current_backwards ? M_PI : 0) +
              ::std::atan2(s.pose.Get()(1) - s_last.pose.Get()(1),
                           s.pose.Get()(0) - s_last.pose.Get()(0));
      EXPECT_ANGLE_NEAR(measured_angle,
                        average_angle(s_last.pose.Get()(2), s.pose.Get()(2)),
                        0.01)
          << "at i=" << i << " " << s.pose.Get()(1) << s_last.pose.Get()(1);

      // Consistency of drivetrain delta position with pose delta position
      EXPECT_NEAR((s.pose.translational() - s_last.pose.translational()).norm() *
                         (backwards ? -1.0 : 1.0),
                  0.5 * (s.drivetrain_state(0) - s_last.drivetrain_state(0) +
                         s.drivetrain_state(2) - s_last.drivetrain_state(2)),
                  0.02)
          << "at i=" << i;

      // Consistency of pose heading and drivetrain heading
      EXPECT_ANGLE_NEAR(s.pose.heading(),
                        (s.drivetrain_state(2) - s.drivetrain_state(0)) * 0.5 /
                              kRadius,
                        0.01)
          << "at i=" << i;

      s_last = s;
    }
    EXPECT_TRUE(s_last.profile_complete);
    EXPECT_NEAR((s_last.drivetrain_state(1) + s_last.drivetrain_state(3)) * 0.5,
                final_velocity, 1e-3);
    EXPECT_NEAR((s_last.drivetrain_state(3) - s_last.drivetrain_state(1)) /
                    (kRadius * 2.0),
                final_angular_velocity, 1e-3);
    EXPECT_NEAR(s_last.distance_remaining, 0, 5e-3);
    EXPECT_NEAR(s_last.time_remaining, 0, 1e-3);
  }

  void Log(const char* filename) {
    trajectory_.SetPath(path_, initial_state_,
                        final_velocity_, final_angular_velocity_);

    ::std::ofstream file(filename);
    for (size_t i = 0; i < 2000; i++) {
      Trajectory::Sample s = trajectory_.Update();
      file << s.drivetrain_state(0) << ',' << s.drivetrain_state(1) << ',' <<
              s.drivetrain_state(2) << ',' << s.drivetrain_state(3) << ',' <<
              s.pose.Get()(0) << ',' << s.pose.Get()(1) << ',' << s.pose.Get()(2) <<
              ',' << s.distance_remaining << ',' << s.time_remaining << ',' <<
              s.profile_complete << std::endl;
    }
  }

 protected:
  HermitePath path_{Pose(), Pose(), 0, 0, false, 0, 0, 0, 0};
  State initial_state_;
  Trajectory trajectory_;
  double final_velocity_;
  double final_angular_velocity_;
};

TEST_F(TrajectoryTest, StraightLine) {
  Pose a = (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << 10.0, 0.0, 0.0).finished();
  Run(a, b);
}

TEST_F(TrajectoryTest, SimpleSCurve) {
  Pose a = (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << 2.0, 2.0, 0.0).finished();
  Run(a, b, 0.0, 0.0, false);
}

TEST_F(TrajectoryTest, DrivesBackwards) {
  Pose a = (Eigen::Vector3d() << 2.0, 2.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();
  Run(a, b, 0.0, 0.0, true);
}

TEST_F(TrajectoryTest, StartAtFollowThrough) {
  Pose a = (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << 3.0, 1.0, 0.0).finished();
  Run(a, b, 1.0, 0.0, false);
}

TEST_F(TrajectoryTest, InitialVelocityBackwards) {
  Pose a = (Eigen::Vector3d() << -3.5, 0.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << -6.65, -1.0, M_PI * -0.3).finished();
  Run(a, b, -3.991, 0, true);
}

TEST_F(TrajectoryTest, ConflictingDirectionality) {
  Pose a = (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << 1.0, 0.0, 0.0).finished();
  Run(a, b, -1.0, 0, false);
}

TEST_F(TrajectoryTest, AngularVelocity) {
  Pose a = (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << 2.0, 2.0, 0.0).finished();
  Run(a, b, 1.0, 0.0, false, -0.2, 0.0, 1.0, 0.0);
}

TEST_F(TrajectoryTest, AngularVelocityReverse) {
  Pose a = (Eigen::Vector3d() << 1.0, 1.0, 1.6).finished();
  Pose b = (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();
  Run(a, b, -1.0, -1.0, true, 0, 0, -1.0, -1.0);
}

TEST_F(TrajectoryTest, FinalVelocity) {
  Pose a = (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << 2.0, 0.0, 0.0).finished();
  Run(a, b, 0.0, 1.0, false, 0, 0, 0, 0.5);
}

TEST_F(TrajectoryTest, ExtraTest1) {
  Pose a = (Eigen::Vector3d() << 3.5, 0.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << 2.2, 3.62, M_PI * -0.2).finished();
  Run(a, b, 3.0, 0.0, false);
}

TEST_F(TrajectoryTest, ExtraTest2) {
  Pose a = (Eigen::Vector3d() << -5.21, -1.37, -1.20).finished();
  Pose b = (Eigen::Vector3d() << -6.8, -1.2, 0.33).finished();
  Run(a, b, 0.0, 0.0, true);
}

TEST_F(TrajectoryTest, StepByAcceleration) {
  auto test_step_acc = [](double d, double v0, double a) {
    double v1 = StepVelocityByAcceleration(d, v0, a);
    double t = 2 * d / (v0 + v1);

    EXPECT_NEAR(v0 * t + .5 * a * t * t, d, 1e-3);
    EXPECT_GT(t, 0);
  };

  test_step_acc(1.0, 0.0, 1.0);
  test_step_acc(1.0, 1.0, -0.5);
  test_step_acc(-1.0, -1.0, -1.5);
  test_step_acc(-1.0, -1.0, 0.5);
}

TEST_F(TrajectoryTest, VelocityAtDirectionChange) {
  auto test_velocity = [](double d1, double d2, double a) {
    double calculate_forwards = VelocityAtDirectionChange(d1, d2, a);
    double calculate_backwards = VelocityAtDirectionChange(-d2, -d1, a);

    EXPECT_NEAR(calculate_forwards, -calculate_backwards, 1e-3);
  };

  test_velocity(1.0, -0.5, 1.0);
  test_velocity(1.0, -1.0, 1.0);
  test_velocity(1.0, -1.5, 1.0);
}

}  // namespace testing
}  // namespace paths
}  // namespace control_loops
}  // namespace frc971
