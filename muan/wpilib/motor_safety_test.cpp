#include "gtest/gtest.h"
#include "motor_safety.h"

TEST(MotorSafetyTest, NeverOverCurrentThresh) {
  MotorSafety safety = MotorSafety(100., 2., 2., 0.005);
  for (double t = 0.005; t < 10; t += 0.005) {
    double current = 30;
    double voltage = 10;
    double safe_voltage = safety.Update(voltage, current);
    EXPECT_NEAR(voltage, safe_voltage, 1e-5);
    EXPECT_FALSE(safety.is_stalled());
  }
}

TEST(MotorSafetyTest, GoesOverCurrentThresh) {
  MotorSafety safety = MotorSafety(100., 3., 2., 0.005);
  for (double t = 0.005; t < 10.0; t += 0.005) {
    double current = 200;
    double voltage = 10;
    double safe_voltage = safety.Update(voltage, current);
    if (t < 3.0) {
      EXPECT_NEAR(safe_voltage, voltage, 1e-5);
      EXPECT_FALSE(safety.is_stalled());
    } else {
      EXPECT_NEAR(safe_voltage, 0, 1e-5);
      EXPECT_TRUE(safety.is_stalled());
    }
  }
}

TEST(MotorSafetyTest, TooShortStall) {
  MotorSafety safety = MotorSafety(100., 2., 2., 0.005);
  for (double t = 0.005; t < 10; t += 0.005) {
    double current = 120 * sin(t);
    double voltage = 10;
    double safe_voltage = safety.Update(voltage, current);
    EXPECT_NEAR(voltage, safe_voltage, 1e-5);
    EXPECT_FALSE(safety.is_stalled());
  }
}

TEST(MotorSafetyTest, CurrentSpikeThenDrop) {
  MotorSafety safety = MotorSafety(100., 3., 2., 0.005);
  for (double t = 0.005; t < 10.0; t += 0.005) {
    double current = t < 4 ? 200 : 90;
    double voltage = 10;
    double safe_voltage = safety.Update(voltage, current);
    if (t < 3.0) {
      EXPECT_NEAR(safe_voltage, voltage, 1e-5);
      EXPECT_FALSE(safety.is_stalled());
    } else if (t < 6.09) {  // Slightly more than 6 seconds to account for the moving average filter getting
                            // track of the sudden current change.
      EXPECT_NEAR(safe_voltage, 0, 1e-5);
      EXPECT_TRUE(safety.is_stalled());
    } else {
      EXPECT_NEAR(safe_voltage, voltage, 1e-5);
      EXPECT_FALSE(safety.is_stalled());
    }
  }
}
