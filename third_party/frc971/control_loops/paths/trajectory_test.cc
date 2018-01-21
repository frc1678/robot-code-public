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

  void Run(Pose initial, Pose final, bool backwards = false, double initial_velocity = 0) {
    initial_ = initial;
    path_ = HermitePath(initial, final, backwards);

    State initial_state = (State() << 0, initial_velocity, 0, initial_velocity).finished();

    trajectory_.SetPath(path_, initial_state);

    Trajectory::Sample s_last = {};
    s_last.pose = initial;
    s_last.drivetrain_state = initial_state;
    s_last.drivetrain_state(2) = initial.heading();
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

      // Acceleration constraints

      // Consistency - does not disagree with itself with respect to arc
      // length/velocity, heading/angular velocity, and heading/velocity/x-y
      // pose
      s_last = s;
    }
  }

  void Log(const char* filename, double initial_velocity = 0) {
    trajectory_.SetPath(path_, (Eigen::Matrix<double, 4, 1>() << 0, initial_velocity, 0, initial_velocity).finished());

    ::std::ofstream file(filename);
    for (size_t i = 0; i < 2000; i++) {
      Trajectory::Sample s = trajectory_.Update();
      file << s.drivetrain_state(0) << ',' << s.drivetrain_state(1) << ',' <<
              s.drivetrain_state(2) << ',' << s.drivetrain_state(3) << ',' <<
              s.pose.Get()(0) << ',' << s.pose.Get()(1) << std::endl;
    }
  }

 protected:
  HermitePath path_{Pose(), Pose(), false};
  Pose initial_;
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

TEST_F(TrajectoryTest, StepByAcceleration) {
  auto test_step_acc = [](double d, double v0, double a) {
    double v1 = StepVelocityByAcceleration(d, v0, a);
    double t = 2 * d / (v0 + v1);

    EXPECT_NEAR(v0 * t + .5 * a * t * t, d, 1e-3);
  };

  test_step_acc(1.0, 0.0, 1.0);
  test_step_acc(1.0, 1.0, -0.5);
  test_step_acc(-1.0, -1.0, -1.5);
  test_step_acc(-1.0, -1.0, 0.5);
}

}  // namespace testing
}  // namespace paths
}  // namespace control_loops
}  // namespace frc971
