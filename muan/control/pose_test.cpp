#include "muan/control/pose.h"
#include "gtest/gtest.h"

namespace muan {
namespace control {

TEST(Pose, HeadingWrapping) {
  Pose a((Eigen::Vector3d() << 1.0, 2.0, 0.0).finished());
  Pose b = a.RotateBy(M_PI);
  EXPECT_NEAR(b.translational()(0), -1.0, 1e-6);
  EXPECT_NEAR(b.translational()(1), -2.0, 1e-6);

  // Either the heading should be -pi or pi, it doesn't matter
  EXPECT_NEAR(::std::abs(b.heading()), M_PI, 1e-6);

  Pose c((Eigen::Vector3d() << 0.0, 0.0, -1.0).finished());
  EXPECT_NEAR((b + c).heading(), M_PI - 1.0, 1e-6);
}

TEST(Pose, Compose) {
  Pose a((Eigen::Vector3d() << 2.0, 0.0, M_PI / 4.0).finished());
  Pose b((Eigen::Vector3d() << 1.0, 2.0, M_PI).finished());
  Pose c = a.Compose(b);

  EXPECT_NEAR(c.translational()(0), 2.0 - ::std::sqrt(2.0) / 2.0, 1e-6);
  EXPECT_NEAR(c.translational()(1), 3.0 * ::std::sqrt(2.0) / 2.0, 1e-6);
  EXPECT_NEAR(c.heading(), -3.0 * M_PI / 4.0, 1e-6);
}

}  // namespace control
}  // namespace muan
