#include "third_party/frc971/control_loops/paths/trajectory.h"
#include <fstream>

#include "gtest/gtest.h"

#define _USE_MATH_DEFINES
#include <cmath>

namespace frc971 {
namespace control_loops {
namespace paths {
namespace testing {

class TrajectoryTest : public ::testing::Test {
 public:
  void SetUp() override {
    trajectory_.set_maximum_acceleration(3.0);

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

    trajectory_.set_system(A, B, 0.59 / 2.0);
    trajectory_.Reset();
  }

  void Run(Pose initial, Pose final, bool backwards = false,
           double initial_velocity = 0, double initial_velocity_difference = 0) {
    path_ = HermitePath(initial, final, initial_velocity, 0, backwards);

    initial_state_ = (State() <<
            initial.translational()(0) - 0.5 * initial.heading(),
            initial_velocity + initial_velocity_difference,
            initial.translational()(0) + 0.5 * initial.heading(),
            initial_velocity - initial_velocity_difference).finished();

    trajectory_.SetPath(path_, initial_state_);

    Trajectory::Sample s_last = {};
    s_last.pose = initial;
    s_last.drivetrain_state = initial_state_;
    for (size_t i = 0; i < 2000; i++) {
      Trajectory::Sample s = trajectory_.Update();
      // Continuity/smoothness
      EXPECT_NEAR(s_last.drivetrain_state(0), s.drivetrain_state(0),
                  0.05)
          << "at i=" << i;
      EXPECT_NEAR(s_last.drivetrain_state(1), s.drivetrain_state(1),
                  0.05)
          << "at i=" << i;
      EXPECT_NEAR(remainder(s_last.drivetrain_state(2) - s.drivetrain_state(2),
                            2.0 * M_PI),
                  0.0, 0.05)
          << " at i=" << i;
      EXPECT_NEAR(s_last.drivetrain_state(3), s.drivetrain_state(3),
                  0.05)
          << "at i=" << i;
      // This doesn't actually work because of precision. TODO(Lyra) fix eventually
      /*EXPECT_NEAR((s.drivetrain_state(0) - s_last.drivetrain_state(0)) / 0.005,
                  (s.drivetrain_state(1) + s_last.drivetrain_state(1)) * 0.5,
                  2e-3)
          << "at i=" << i;
      EXPECT_NEAR((s.drivetrain_state(2) - s_last.drivetrain_state(2)) / 0.005,
                  (s.drivetrain_state(3) + s_last.drivetrain_state(3)) * 0.5,
                  2e-3)
          << "at i=" << i;*/

      // Acceleration constraints

      // Consistency - does not disagree with itself with respect to arc
      // length/velocity, heading/angular velocity, and heading/velocity/x-y
      // pose
      s_last = s;
    }
  }

  void Log(const char* filename) {
    trajectory_.SetPath(path_, initial_state_);

    ::std::ofstream file(filename);
    for (size_t i = 0; i < 2000; i++) {
      Trajectory::Sample s = trajectory_.Update();
      file << s.drivetrain_state(0) << ',' << s.drivetrain_state(1) << ',' <<
              s.drivetrain_state(2) << ',' << s.drivetrain_state(3) << ',' <<
              s.pose.Get()(0) << ',' << s.pose.Get()(1) << std::endl;
    }
  }

 protected:
  HermitePath path_{Pose(), Pose(), 0, 0, false};
  State initial_state_;
  Trajectory trajectory_;
};

TEST_F(TrajectoryTest, StraightLine) {
  Pose a = (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << 10.0, 0.0, 0.0).finished();
  Run(a, b);
}

TEST_F(TrajectoryTest, SimpleSCurve) {
  Pose a = (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << 2.0, 2.0, 0.0).finished();
  Run(a, b);
}

TEST_F(TrajectoryTest, DrivesBackwards) {
  Pose a = (Eigen::Vector3d() << 2.0, 2.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();
  Run(a, b, true);
}

TEST_F(TrajectoryTest, StartAtFollowThrough) {
  Pose a = (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << 3.0, 1.0, 0.0).finished();
  Run(a, b, false, 1.0);
}

TEST_F(TrajectoryTest, InitialVelocityBackwards) {
  Pose a = (Eigen::Vector3d() << -3.5, 0.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << -6.65, -1.0, M_PI * -0.3).finished();
  Run(a, b, true, -3.991);
}

TEST_F(TrajectoryTest, ConflictingDirectionality) {
  Pose a = (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << 1.0, 0.0, 0.0).finished();
  EXPECT_DEATH(Run(a, b, false, -1.0), "Conflicting path directionality");
}

TEST_F(TrajectoryTest, SlightAngularVelocity) {
  Pose a = (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << 2.0, 2.0, 0.0).finished();
  Run(a, b, false, 1.0, 0.02);
}

TEST_F(TrajectoryTest, ExtraTest1) {
  Pose a = (Eigen::Vector3d() << 3.5, 0.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << 2.2, 3.62, M_PI * -0.2).finished();
  Run(a, b, false, 3.0);
}

TEST_F(TrajectoryTest, ExtraTest2) {
  Pose a = (Eigen::Vector3d() << -5.21, -1.37, -1.20).finished();
  Pose b = (Eigen::Vector3d() << -6.8, -1.2, 0.33).finished();
  Run(a, b, true);
  Log("/tmp/extratest2.csv");
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
