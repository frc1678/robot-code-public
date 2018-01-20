#include "third_party/frc971/control_loops/paths/path.h"
#include <fstream>

#include "gtest/gtest.h"

#define _USE_MATH_DEFINES
#include <cmath>

namespace frc971 {
namespace control_loops {
namespace paths {
namespace testing {

TEST(Pose, HeadingWrapping) {
  Pose a = (Eigen::Vector3d() << 1.0, 2.0, 0.0).finished();
  Pose b = a.RotateBy(M_PI);
  ASSERT_NEAR(b.translational()(0), -1.0, 1e-6);
  ASSERT_NEAR(b.translational()(1), -2.0, 1e-6);

  // Either the heading should be -pi or pi, it doesn't matter
  ASSERT_NEAR(::std::abs(b.heading()), M_PI, 1e-6);

  Pose c = (Eigen::Vector3d() << 0.0, 0.0, -1.0).finished();
  ASSERT_NEAR((b + c).heading(), M_PI - 1.0, 1e-6);
}

TEST(Pose, Compose) {
  Pose a = (Eigen::Vector3d() << 2.0, 0.0, M_PI / 4.0).finished();
  Pose b = (Eigen::Vector3d() << 1.0, 2.0, M_PI).finished();
  Pose c = a.Compose(b);

  ASSERT_NEAR(c.translational()(0), 2.0 - ::std::sqrt(2.0) / 2.0, 1e-6);
  ASSERT_NEAR(c.translational()(1), 3.0 * ::std::sqrt(2.0) / 2.0, 1e-6);
  ASSERT_NEAR(c.heading(), -3.0 * M_PI / 4.0, 1e-6);
}

class HermiteSplineTest : public ::testing::Test {
 public:
  static constexpr size_t kNumSamples = 1001;

  void Set() {}

  void Run(Pose initial, Pose final, bool backwards = false) {
    path_ = HermitePath(initial, final, backwards);
    path_.Populate(0.0, 1.0, &poses_[0], poses_.size());

    EXPECT_NEAR(poses_[0].translational()(0), initial.translational()(0), 1e-6);
    EXPECT_NEAR(poses_[0].translational()(1), initial.translational()(1), 1e-6);
    EXPECT_NEAR(poses_[0].heading(), initial.heading(), 1e-6);

    for (size_t i = 1; i < kNumSamples; i++) {
      // Continuity/smoothness
      EXPECT_NEAR(
          (poses_[i] - poses_[i - 1]).translational().lpNorm<Eigen::Infinity>(),
          0.0, 1e-2)
          << "(Failure exists at i=" << i << ")";
      EXPECT_NEAR((poses_[i] - poses_[i - 1]).heading(), 0.0, 3e-2)
          << "(Failure exists at i=" << i << ")";
      // Accuracy: heading should be the same as heading calculated from
      // delta-xy
      Position delta = (poses_[i] - poses_[i - 1]).translational();
      if (backwards) {
        delta *= -1;
      }
      EXPECT_NEAR(
          remainder(poses_[i].heading() - ::std::atan2(delta(1), delta(0)),
                    2.0 * M_PI),
          0.0, 1e-1)
          << "(Failure exists at i=" << i << ")";
    }

    EXPECT_NEAR(poses_[kNumSamples - 1].translational()(0),
                final.translational()(0), 1e-6);
    EXPECT_NEAR(poses_[kNumSamples - 1].translational()(1),
                final.translational()(1), 1e-6);
    EXPECT_NEAR(poses_[kNumSamples - 1].heading(), final.heading(), 1e-6);
  }

  void Log(const char* filename) {
    ::std::ofstream file(filename);
    for (size_t i = 0; i < kNumSamples; i++) {
      auto p = poses_[i].Get();
      file << p(0) << ',' << p(1) << ',' << p(2) << std::endl;
    }
  }

 protected:
  HermitePath path_{Pose(), Pose()};
  ::std::array<Pose, kNumSamples> poses_;
};

TEST_F(HermiteSplineTest, StraightLine) {
  Pose a = (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << 1.0, 0.0, 0.0).finished();
  Run(a, b);
}

TEST_F(HermiteSplineTest, SimpleSCurve) {
  Pose a = (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << 1.0, 1.0, 0.0).finished();
  Run(a, b);
}

TEST_F(HermiteSplineTest, Reversed) {
  Pose a = (Eigen::Vector3d() << 1.0, 1.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();
  Run(a, b);
}

TEST_F(HermiteSplineTest, HeadingBackwards) {
  Pose a = (Eigen::Vector3d() << 0.0, 0.0, M_PI / 2.0).finished();
  Pose b = (Eigen::Vector3d() << -1.0, -1.0, -M_PI / 2.0).finished();
  Run(a, b);
}

TEST_F(HermiteSplineTest, Wraparound) {
  // This test crosses theta=+-pi twice, once in each direction
  Pose a = (Eigen::Vector3d() << 0.0, 0.0, M_PI / 2).finished();
  Pose b = (Eigen::Vector3d() << -1.0, 0.0, M_PI / 2).finished();
  Run(a, b);
}

TEST_F(HermiteSplineTest, DrivesBackwards) {
  Pose a = (Eigen::Vector3d() << 1.0, 1.0, 0.0).finished();
  Pose b = (Eigen::Vector3d() << 0.0, 0.0, 0.0).finished();
  Run(a, b, true);
}

}  // namespace testing
}  // namespace paths
}  // namespace control_loops
}  // namespace frc971
