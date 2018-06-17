#include "muan/phoenix/mock/victor_wrapper.h"
#include "gtest/gtest.h"

namespace muan {
namespace phoenix {

TEST(VictorWrapper, SetPosition) {
  VictorWrapper victor(0, VictorWrapper::Config());

  victor.SetPositionGoal(0., 0.);
  EXPECT_NEAR(victor.position(), 0., 1e-9);

  victor.SetPositionGoal(1., 0.);
  EXPECT_NEAR(victor.position(), 1, 1e-9);
  EXPECT_NEAR(victor.velocity(), 1 / (10 * ms), 1e-9);
}

TEST(VictorWrapper, SetVelocity) {
  VictorWrapper victor(0, VictorWrapper::Config());

  victor.SetPositionGoal(0., 0.);
  EXPECT_NEAR(victor.position(), 0., 1e-9);

  victor.SetVelocityGoal(1., 0.);
  EXPECT_NEAR(victor.velocity(), 1, 1e-9);
  EXPECT_NEAR(victor.position(), 10 * ms, 1e-9);
}

TEST(VictorWrapper, Calibrate) {
  VictorWrapper victor(0, VictorWrapper::Config());

  victor.SetPositionGoal(0., 0.);
  EXPECT_NEAR(victor.position(), 0., 1e-9);

  victor.ResetSensor(1.);
  EXPECT_NEAR(victor.position(), 1., 1e-9);
  EXPECT_NEAR(victor.velocity(), 0., 1e-9);
}

}  // namespace phoenix
}  // namespace muan
