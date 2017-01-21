#include "gtest/gtest.h"
#include "motor_safety.h"
#include <math.h>

TEST(MotorSafetyTest, NeverOverCurrentThresh) {
  muan::wpilib::MotorSafety safety = muan::wpilib::MotorSafety(100., 2., 2., 0.005);
  for (double t = 0.005; t < 10; t += 0.005) {
    double current = 30;
    double voltage = 10;
    double safe_voltage = safety.Update(voltage, current);
    EXPECT_NEAR(voltage, safe_voltage, 1e-5);
    EXPECT_FALSE(safety.is_stalled());
  }
}

TEST(MotorSafetyTest, GoesOverCurrentThresh) {
  muan::wpilib::MotorSafety safety = muan::wpilib::MotorSafety(100., 3., 2., 0.005);
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
  muan::wpilib::MotorSafety safety = muan::wpilib::MotorSafety(100., 2., 2., 0.005);
  for (double t = 0.005; t < 10; t += 0.005) {
    double current = 120 * sin(t);
    double voltage = 10;
    double safe_voltage = safety.Update(voltage, current);
    EXPECT_NEAR(voltage, safe_voltage, 1e-5);
    EXPECT_FALSE(safety.is_stalled());
  }
}

TEST(MotorSafetyTest, CurrentSpikeThenDrop) {
  muan::wpilib::MotorSafety safety = muan::wpilib::MotorSafety(100., 3., 2., 0.005);
  for (double t = 0.005; t < 10.0; t += 0.005) {
    double current = t < 4 ? 200 : 90;
    double voltage = 10;
    double safe_voltage = safety.Update(voltage, current);
    if (t < 3.0) {
      EXPECT_NEAR(safe_voltage, voltage, 1e-5);
      EXPECT_FALSE(safety.is_stalled());
    } else if (t < 6.095) {  // Slightly more than 6 seconds to account for the moving average filter getting
                             // track of the sudden current change.
      EXPECT_NEAR(safe_voltage, 0, 1e-5);
      EXPECT_TRUE(safety.is_stalled());
    } else {
      EXPECT_NEAR(safe_voltage, voltage, 1e-5);
      EXPECT_FALSE(safety.is_stalled());
    }
  }
}

TEST(MotorSafetyTest, InfiniteReset) {
  muan::wpilib::MotorSafety safety = muan::wpilib::MotorSafety(100., 2., std::numeric_limits<int>::max(), 0.01);
  for (double t = 0.01; t < 100; t += 0.01) {
    double current = t < 4 ? 200 : 90;
    double voltage = 10;
    double safe_voltage = safety.Update(voltage, current);
    if (t < 2.0) {
      EXPECT_NEAR(safe_voltage, voltage, 1e-5);
      EXPECT_FALSE(safety.is_stalled());
    } else {
      EXPECT_NEAR(safe_voltage, 0, 1e-5);
      EXPECT_TRUE(safety.is_stalled());
    }
  }
}

TEST(MotorSafetyTest, SlowStall) {
  muan::wpilib::MotorSafety safety = muan::wpilib::MotorSafety(100., 1., 1., 0.005);
  for (double t = 0.005; t < 9; t += 0.005) {
    double current = t < 4 ? 200 : t < 4.5 ? 20 : t < 6 ? 200 : 20; // Makes the current fluctuate back above the threshold before it is done resetting
    double voltage = 10;
    double safe_voltage = safety.Update(voltage, current);
    if(t < 1.0) {
      EXPECT_NEAR(safe_voltage, voltage, 1e-5);
      EXPECT_FALSE(safety.is_stalled());
    } else if (t < 7.055) { // A little more to allow the moving average to catch up on current
      EXPECT_NEAR(safe_voltage, 0, 1e-5);
      EXPECT_TRUE(safety.is_stalled());
    } else {
      EXPECT_NEAR(safe_voltage, voltage, 1e-5);
      EXPECT_FALSE(safety.is_stalled());
    }
  }
}
