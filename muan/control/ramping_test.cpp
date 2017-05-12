#include "muan/control/ramping.h"
#include "gtest/gtest.h"

TEST(Ramping, RampsUp) {
  muan::control::Ramping ramping{0.05, 0.0, false};
  // Loops in tests are 1-indexed because ramping occurs immediately
  for (int i = 1; i <= 20; i++) {
    EXPECT_NEAR(ramping.Update(1.0), i * 0.05, 1e-4);
  }

  for (int i = 0; i < 10; i++) {
    EXPECT_NEAR(ramping.Update(1.0), 1.0, 1e-4);
  }
}

TEST(Ramping, DropsDown) {
  muan::control::Ramping ramping{0.05, 0.0, false};
  for (int i = 1; i <= 20; i++) {
    EXPECT_NEAR(ramping.Update(1.0), i * 0.05, 1e-4);
  }

  for (int i = 0; i < 10; i++) {
    EXPECT_NEAR(ramping.Update(0.0), 0.0, 1e-4);
  }
}

TEST(Ramping, RampsSmoothlyDown) {
  muan::control::Ramping ramping{0.05, 0.0, true};
  for (int i = 1; i <= 20; i++) {
    EXPECT_NEAR(ramping.Update(1.0), i * 0.05, 1e-4);
  }

  for (int i = 1; i < 20; i++) {
    EXPECT_NEAR(ramping.Update(0.0), (20 - i) * 0.05, 1e-4);
  }
}
