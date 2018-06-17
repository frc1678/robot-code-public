#include "muan/phoenix/mock/talon_wrapper.h"
#include "gtest/gtest.h"

namespace muan {
namespace phoenix {

TEST(TalonWrapper, SetPosition) {
  TalonWrapper talon(0, TalonWrapper::Config());

  talon.SetPositionGoal(0., 0.);
  EXPECT_NEAR(talon.position(), 0., 1e-9);

  talon.SetPositionGoal(1., 0.);
  EXPECT_NEAR(talon.position(), 1, 1e-9);
  EXPECT_NEAR(talon.velocity(), 1 / (10 * ms), 1e-9);
}

TEST(TalonWrapper, SetVelocity) {
  TalonWrapper talon(0, TalonWrapper::Config());

  talon.SetPositionGoal(0., 0.);
  EXPECT_NEAR(talon.position(), 0., 1e-9);

  talon.SetVelocityGoal(1., 0.);
  EXPECT_NEAR(talon.velocity(), 1, 1e-9);
  EXPECT_NEAR(talon.position(), 10 * ms, 1e-9);
}

TEST(TalonWrapper, Calibrate) {
  TalonWrapper talon(0, TalonWrapper::Config());

  talon.SetPositionGoal(0., 0.);
  EXPECT_NEAR(talon.position(), 0., 1e-9);

  talon.ResetSensor(1.);
  EXPECT_NEAR(talon.position(), 1., 1e-9);
  EXPECT_NEAR(talon.velocity(), 0., 1e-9);
}

}  // namespace phoenix
}  // namespace muan
