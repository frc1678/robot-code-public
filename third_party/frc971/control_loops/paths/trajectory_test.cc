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
    trajectory_.set_maximum_acceleration(1.0);
    trajectory_.set_maximum_velocity(1.0);
    trajectory_.set_maximum_angular_velocity(1.0);
    trajectory_.set_maximum_angular_acceleration(1.0);
    trajectory_.Reset();
  }

  void Run(Pose initial, Pose final) {
    path_ = HermitePath(initial, final);
    trajectory_.SetPath(path_);

    Trajectory::Sample s_last = {};
    s_last.pose = initial;
    s_last.drivetrain_state = State::Zero();
    s_last.drivetrain_state(2) = initial.heading();
    for (size_t i = 0; i < 1000; i++) {
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
          << "s_last.drivetrain_state(2) evaluates to "
          << s_last.drivetrain_state(2)
          << " and s.drivetrain_state(2) evaluates to " << s.drivetrain_state(2)
          << " at i=" << i;

      // Acceleration constraints

      // Consistency - does not disagree with itself with respect to arc
      // length/velocity, heading/angular velocity, and heading/velocity/x-y
      // pose
      s_last = s;
    }
  }

 protected:
  HermitePath path_{Pose(), Pose()};
  Trajectory trajectory_;
};

TEST_F(TrajectoryTest, StraightLine) {
  Pose a = (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << 1.0, 0.0, 0.0).finished();
  Run(a, b);
}

TEST_F(TrajectoryTest, SimpleSCurve) {
  Pose a = (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << 1.0, 1.0, 0.0).finished();
  Run(a, b);
}

TEST_F(TrajectoryTest, Reversed) {
  // This test and SimpleSCurve form a closed path in R^2, so one of them must
  // cross the theta=+-pi boundary somewhere...
  Pose a = (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << -1.0, -1.0, 0.0).finished();
  Run(a, b);
}

TEST_F(TrajectoryTest, HeadingBackwards) {
  Pose a = (Eigen::Vector3d() << 0.0, 0.0, M_PI / 2.0).finished();
  Pose b = (Eigen::Vector3d() << -1.0, -1.0, -M_PI / 2.0).finished();
  Run(a, b);

  ::std::ofstream file("/tmp/file.csv");
  for (size_t i = 0; i < 5000; i++) {
    Trajectory::Sample s = trajectory_.Update();
    file << i * 0.001 << ',' << s.drivetrain_state(0) << ','
         << s.drivetrain_state(1) << ',' << s.drivetrain_state(2) << ','
         << s.drivetrain_state(3) << std::endl;
  }
}

}  // namespace testing
}  // namespace paths
}  // namespace control_loops
}  // namespace frc971
