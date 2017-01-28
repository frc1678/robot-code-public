#include <math.h>
#include <limits>
#include "gtest/gtest.h"
#include "muan/utils/monitor.h"

TEST(MotorSafetyTest, NeverOverCurrentThresh) {
  muan::utils::Monitor safety = muan::utils::Monitor(100., 2., 2., 0.005);
  for (int i = 0; i < 2000; i++) {
    double current = 30;
    double voltage = 10;
    double safe_voltage = safety.Update(voltage, current);
    EXPECT_NEAR(voltage, safe_voltage, 1e-5);
    EXPECT_FALSE(safety.is_at_thresh());
  }
}

TEST(MotorSafetyTest, GoesOverCurrentThresh) {
  muan::utils::Monitor safety = muan::utils::Monitor(100., 3., 2., 0.005);
  for (int i = 0; i < 2000; i++) {
    double t = i * 0.005;
    double current = 200;
    double voltage = 10;
    double safe_voltage = safety.Update(voltage, current);
    if (t < 3.0) {
      EXPECT_NEAR(safe_voltage, voltage, 1e-5);
      EXPECT_FALSE(safety.is_at_thresh());
    } else {
      EXPECT_NEAR(safe_voltage, 0, 1e-5);
      EXPECT_TRUE(safety.is_at_thresh());
    }
  }
}

TEST(MotorSafetyTest, TooShortStall) {
  muan::utils::Monitor safety = muan::utils::Monitor(100., 2., 2., 0.005);
  for (int i = 0; i < 2000; i++) {
    double t = i * 0.005;
    double current = 120 * sin(t);
    double voltage = 10;
    double safe_voltage = safety.Update(voltage, current);
    EXPECT_NEAR(voltage, safe_voltage, 1e-5);
    EXPECT_FALSE(safety.is_at_thresh());
  }
}

TEST(MotorSafetyTest, CurrentSpikeThenDrop) {
  muan::utils::Monitor safety = muan::utils::Monitor(100., 3., 2., 0.005);
  for (int i = 0; i < 2000; i++) {
    double t = i * 0.005;
    double current = t < 4 ? 200 : 90;
    double voltage = 10;
    double safe_voltage = safety.Update(voltage, current);
    if (t < 3.0) {
      EXPECT_NEAR(safe_voltage, voltage, 1e-5);
      EXPECT_FALSE(safety.is_at_thresh());
    } else if (t < 6.090) {  // Slightly more than 6 seconds to account for the moving average filter getting
                             // track of the sudden current change.
      EXPECT_NEAR(safe_voltage, 0, 1e-5);
      EXPECT_TRUE(safety.is_at_thresh());
    } else {
      EXPECT_NEAR(safe_voltage, voltage, 1e-5);
      EXPECT_FALSE(safety.is_at_thresh());
    }
  }
}

TEST(MotorSafetyTest, InfiniteReset) {
  muan::utils::Monitor safety = muan::utils::Monitor(100., 2., std::numeric_limits<double>::max(), 0.01);
  for (int i = 0; i < 10000; i++) {
    double t = i * 0.01;
    double current = t < 4 ? 200 : 90;
    double voltage = 10;
    double safe_voltage = safety.Update(voltage, current);
    if (t < 1.990) {
      EXPECT_NEAR(safe_voltage, voltage, 1e-5);
      EXPECT_FALSE(safety.is_at_thresh());
    } else {
      EXPECT_NEAR(safe_voltage, 0, 1e-5);
      EXPECT_TRUE(safety.is_at_thresh());
    }
  }
}

TEST(MotorSafetyTest, SlowStall) {
  muan::utils::Monitor safety = muan::utils::Monitor(100., 1., 1., 0.005);
  for (int i = 0; i < 1600; i++) {
    double t = i * 0.005;
    double current = t < 4 ? 200 : t < 4.5 ? 20 : t < 6 ? 200 : 20;  // Makes the current fluctuate back above
                                                                     // the threshold before it is done
                                                                     // resetting
    double voltage = 10;
    double safe_voltage = safety.Update(voltage, current);
    if (t < 0.995) {
      EXPECT_NEAR(safe_voltage, voltage, 1e-5);
      EXPECT_FALSE(safety.is_at_thresh());
    } else if (t < 7.050) {  // A little more to allow the moving average to catch up on current
      EXPECT_NEAR(safe_voltage, 0, 1e-5);
      EXPECT_TRUE(safety.is_at_thresh());
    } else {
      EXPECT_NEAR(safe_voltage, voltage, 1e-5);
      EXPECT_FALSE(safety.is_at_thresh());
    }
  }
}

TEST(MonitorTest, BelowThreshold) {
  muan::utils::Monitor monitor = muan::utils::Monitor(100., 1., 1., 0.005, false);
  for (int i = 0; i < 1800; i++) {
    double t = i * 0.005;
    double value = t < 2 ? 200 : 20;
    double voltage = 10;
    double safe_voltage = monitor.Update(voltage, value);
    if (t < 3.050) {
      EXPECT_NEAR(safe_voltage, voltage, 1e-5);
      EXPECT_FALSE(monitor.is_at_thresh());
    } else {  // A little more to allow the moving average to catch up on value
      EXPECT_NEAR(safe_voltage, 0, 1e-5);
      EXPECT_TRUE(monitor.is_at_thresh());
    }
  }
}

TEST(MonitorTest, CustomStandingVoltage) {
  muan::utils::Monitor monitor = muan::utils::Monitor(100., 1., 1., 0.005, false, 5);
  for (int i = 0; i < 1800; i++) {
    double t = i * 0.005;
    double value = t < 2 ? 200 : 20;
    double voltage = 10;
    double safe_voltage = monitor.Update(voltage, value);
    if (t < 3.050) {
      EXPECT_NEAR(safe_voltage, voltage, 1e-5);
      EXPECT_FALSE(monitor.is_at_thresh());
    } else {  // A little more to allow the moving average to catch up on value
      EXPECT_NEAR(safe_voltage, 5, 1e-5);
      EXPECT_TRUE(monitor.is_at_thresh());
    }
  }
}

TEST(MonitorTest, CustomHistorySize) {
  muan::utils::Monitor monitor = muan::utils::Monitor(100., 1., 1., 0.005, false, 0, 30);
  for (int i = 0; i < 1800; i++) {
    double t = i * 0.005;
    double value = t < 2 ? 200 : 20;
    double voltage = 10;
    double safe_voltage = monitor.Update(voltage, value);
    if (t < 3.075) {
      EXPECT_NEAR(safe_voltage, voltage, 1e-5);
      EXPECT_FALSE(monitor.is_at_thresh());
    } else {  // A little more to allow the moving average to catch up on value
      EXPECT_NEAR(safe_voltage, 0, 1e-5);
      EXPECT_TRUE(monitor.is_at_thresh());
    }
  }
}

TEST(MonitorTest, Resets) {
  muan::utils::Monitor monitor =
      muan::utils::Monitor(100., 1., std::numeric_limits<double>::max(), 0.005, false, 5);
  for (int i = 0; i < 1800; i++) {
    double t = i * 0.005;
    double value = t < 2 ? 200 : t < 6 ? 20 : 200;
    double voltage = 10;
    double safe_voltage = monitor.Update(voltage, value);
    if (t < 3.050) {
      EXPECT_NEAR(safe_voltage, voltage, 1e-5);
      EXPECT_FALSE(monitor.is_at_thresh());
    } else if (t < 6) {  // A little more to allow the moving average to catch up on value
      EXPECT_NEAR(safe_voltage, 5, 1e-5);
      EXPECT_TRUE(monitor.is_at_thresh());
    } else if (t < 6.007) {
      monitor.Reset();
    } else {
      EXPECT_NEAR(safe_voltage, voltage, 1e-5);
      EXPECT_FALSE(monitor.is_at_thresh());
    }
  }
}

TEST(MonitorTest, CustomTimestep) {
  muan::utils::Monitor monitor = muan::utils::Monitor(100., 2., 2., 0.01);
  for (int i = 0; i < 700; i++) {
    double t = i * 0.01;
    double current = t < 4 ? 200 : 90;
    double voltage = 10;
    double safe_voltage = monitor.Update(voltage, current);
    if (t < 1.990) {
      EXPECT_NEAR(safe_voltage, voltage, 1e-5);
      EXPECT_FALSE(monitor.is_at_thresh());
    } else if (t < 6.17) {
      EXPECT_NEAR(safe_voltage, 0, 1e-5);
      EXPECT_TRUE(monitor.is_at_thresh());
    } else {
      EXPECT_NEAR(safe_voltage, 10, 1e-5);
      EXPECT_FALSE(monitor.is_at_thresh());
    }
  }
}
