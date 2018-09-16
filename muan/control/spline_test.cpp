#include "muan/control/spline.h"
#include <fstream>
#include "gtest/gtest.h"

namespace muan {
namespace control {

class HermiteSplineTest : public ::testing::Test {
 public:
  static constexpr int kNumSamples = 1001;

  void Set() {}

  void Run(Pose initial, Pose final, double initial_velocity = 0,
           double final_velocity = 0, bool backwards = false,
           double extra_distance_initial = 0, double extra_distance_final = 0,
           double initial_angular_velocity = 0,
           double final_angular_velocity = 0) {
    path_ =
        HermiteSpline(initial, final, initial_velocity, final_velocity,
                      backwards, extra_distance_initial, extra_distance_final,
                      initial_angular_velocity, final_angular_velocity);
    poses_ = path_.Populate(0.0, 1.0);

    EXPECT_NEAR(poses_[0].translational()(0), initial.translational()(0), 1e-6);
    EXPECT_NEAR(poses_[0].translational()(1), initial.translational()(1), 1e-6);
    EXPECT_NEAR(poses_[0].heading(), initial.heading(), 1e-6);

    for (int i = 1; i < kNumSamples; i++) {
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

    EXPECT_NEAR(poses_.back().translational()(0), final.translational()(0),
                1e-6);
    EXPECT_NEAR(poses_.back().translational()(1), final.translational()(1),
                1e-6);
    EXPECT_NEAR(poses_.back().heading(), final.heading(), 1e-6);
  }

  void Log(const char* filename) {
    ::std::ofstream file(filename);
    for (size_t i = 0; i < kNumSamples; i++) {
      auto p = poses_[i].Get();
      file << p(0) << ',' << p(1) << ',' << p(2) << std::endl;
    }
  }

 protected:
  HermiteSpline path_{Pose(), Pose(), 0, 0, false, 0, 0, 0, 0};
  std::array<PoseWithCurvature, kNumSamples> poses_;
};

TEST_F(HermiteSplineTest, StraightLine) {
  Pose a((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Pose b((Eigen::Vector3d() << 3.0, 0.0, 0.0).finished());
  Run(a, b);
}

TEST_F(HermiteSplineTest, SimpleSCurve) {
  Pose a((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Pose b((Eigen::Vector3d() << 1.0, 1.0, 0.0).finished());
  Run(a, b);
}

TEST_F(HermiteSplineTest, Reversed) {
  Pose a((Eigen::Vector3d() << 1.0, 1.0, 0.0).finished());
  Pose b((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Run(a, b);
}

TEST_F(HermiteSplineTest, HeadingBackwards) {
  Pose a((Eigen::Vector3d() << 0.0, 0.0, M_PI / 2.0).finished());
  Pose b((Eigen::Vector3d() << -1.0, -1.0, -M_PI / 2.0).finished());
  Run(a, b);
}

TEST_F(HermiteSplineTest, Wraparound) {
  // This test crosses theta=+-pi twice, once in each direction
  Pose a((Eigen::Vector3d() << 0.0, 0.0, M_PI / 2).finished());
  Pose b((Eigen::Vector3d() << -1.0, 0.0, M_PI / 2).finished());
  Run(a, b);
}

TEST_F(HermiteSplineTest, DrivesBackwards) {
  Pose a((Eigen::Vector3d() << 1.0, 1.0, 0.0).finished());
  Pose b((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Run(a, b, 0, 0, true);
}

TEST_F(HermiteSplineTest, ExtraDistance) {
  Pose a((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Pose b((Eigen::Vector3d() << 2.0, 2.0, 0.0).finished());
  Run(a, b, 0, 0, false, 1.0, 0);
}

TEST_F(HermiteSplineTest, InitialVelocity) {
  Pose a((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Pose b((Eigen::Vector3d() << -6.65, -1.0, M_PI * -0.3).finished());
  Run(a, b, 0.3991, 0, true);
}

TEST_F(HermiteSplineTest, AngularVelocity) {
  Pose a((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Pose b((Eigen::Vector3d() << 3.0, 0.0, 0.0).finished());
  Run(a, b, 2, 1, false, 0.5, 0.5, 1, -1);
}

TEST_F(HermiteSplineTest, AngularVelocityReverse) {
  Pose a((Eigen::Vector3d() << 2.0, 1.0, 1.6).finished());
  Pose b((Eigen::Vector3d() << 0.0, 0.0, 0.0).finished());
  Run(a, b, -1, -1, true, 0, 0, -1.6, -1.6);
}

TEST_F(HermiteSplineTest, ExtraTest) {
  Pose a((Eigen::Vector3d() << -6.40, -0.8, 0.5).finished());
  Pose b((Eigen::Vector3d() << -5.0, -1.2, -0.38).finished());
  Run(a, b, 0, 0, false);
}

}  // namespace control
}  // namespace muan
