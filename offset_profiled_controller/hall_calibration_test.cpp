#include "hall_calibration.h"
#include "gtest/gtest.h"

/*
 * Make sure that it initializes to not calibrated
 */
TEST(HallCalibration, Initializes) {
  muan::HallCalibration c(0);
  EXPECT_FALSE(c.Calibrated());
}

/*
 * All calibration tests use 100 to 200 as the magnet's range.
 *
 * Test that it calibrates going up, using all increasing values
 */
TEST(HallCalibration, CalibratesGoingUp) {
  muan::HallCalibration c(0);
  for (int i = 0; i < 100; i++) {
    c.Update(i, false);
    EXPECT_FALSE(c.Calibrated());
  }
  for (int i = 100; i < 200; i++) {
    c.Update(i, true);
    EXPECT_FALSE(c.Calibrated());
  }
  c.Update(200, false);
  EXPECT_TRUE(c.Calibrated());
}

/*
 * Test that it calibrates going down, using all decreasing values
 */
TEST(HallCalibration, CalibratesGoingDown) {
  muan::HallCalibration c(0);
  for (int i = 300; i > 200; i--) {
    c.Update(i, false);
    EXPECT_FALSE(c.Calibrated());
  }
  for (int i = 200; i > 100; i--) {
    c.Update(i, true);
    EXPECT_FALSE(c.Calibrated());
  }
  c.Update(100, false);
  EXPECT_TRUE(c.Calibrated());
}

/*
 * Starting from outside the magnet's range, test that going into the magnet's
 * range and then reversing back out does not calibrate it, and that going
 * through the magnet's range after that successfully calibrates it.
 */
TEST(HallCalibration, ReverseFromOutside) {
  muan::HallCalibration c(0);
  for (int i = 0; i < 100; i++) {
    c.Update(i, false);
    EXPECT_FALSE(c.Calibrated());
  }
  for (int i = 100; i < 150; i++) {
    c.Update(i, true);
    EXPECT_FALSE(c.Calibrated());
  }
  for (int i = 150; i > 100; i--) {
    c.Update(i, true);
    EXPECT_FALSE(c.Calibrated());
  }
  for (int i = 100; i > 0; i--) {
    c.Update(i, false);
    EXPECT_FALSE(c.Calibrated());
  }
  for (int i = 0; i < 100; i++) {
    c.Update(i, false);
    EXPECT_FALSE(c.Calibrated());
  }
  for (int i = 100; i < 200; i++) {
    c.Update(i, true);
    EXPECT_FALSE(c.Calibrated());
  }
  c.Update(200, false);
  EXPECT_TRUE(c.Calibrated());
}

/*
 * Test that starting in the magnet's range and moving out does not calibrate
 * it, and that going through the magnet's range after that successfully
 * calibrates it.
 */
TEST(HallCalibration, ReverseFromInside) {
  muan::HallCalibration c(0);
  for (int i = 150; i > 100; i--) {
    c.Update(i, true);
    EXPECT_FALSE(c.Calibrated());
  }
  for (int i = 100; i > 0; i--) {
    c.Update(i, false);
    EXPECT_FALSE(c.Calibrated());
  }
  for (int i = 0; i < 100; i++) {
    c.Update(i, false);
    EXPECT_FALSE(c.Calibrated());
  }
  for (int i = 100; i < 200; i++) {
    c.Update(i, true);
    EXPECT_FALSE(c.Calibrated());
  }
  c.Update(200, false);
  EXPECT_TRUE(c.Calibrated());
}

/*
 * Test that the calibration routine finds the center of the magnet's range
 * under normal conditions.
 */
TEST(HallCalibration, FindsMagnetCenter) {
  muan::HallCalibration c(0);
  for (int i = 0; i < 100; i++) {
    c.Update(i, false);
  }
  for (int i = 100; i < 200; i++) {
    c.Update(i, true);
  }
  c.Update(200, false);
  ASSERT_TRUE(c.Calibrated());
  EXPECT_NEAR(c.Update(150, true), 0, 1);
}

/*
 * Test that a nonzero value can be used as the center of the magnet
 */
TEST(HallCalibration, UsesMagnetPosition) {
  muan::HallCalibration c(1000);
  for (int i = 0; i < 100; i++) {
    c.Update(i, false);
  }
  for (int i = 100; i < 200; i++) {
    c.Update(i, true);
  }
  c.Update(200, false);
  ASSERT_TRUE(c.Calibrated());
  EXPECT_NEAR(c.Update(150, true), 1000, 1);
}

/*
 * Test that lack of rigidity does not make the calibration get too far off.
 * This is an issue because it can cause the sensors to be turning different
 * directions when the mechanism reverses quickily. Various tests are required
 * to cover this.
 *
 * SensorInaccuracies1 tests that when the hall sensor passes completely by the
 * magnet and the main sensor doesn't, calibration is not affected
 */
TEST(HallCalibration, SensorInaccuracies1) {
  muan::HallCalibration c(0);
  for (int i = 0; i < 100; i++) {
    c.Update(i, false);
  }
  for (int i = 100; i < 195; i++) {
    c.Update(i, true);
  }
  // The main sensor has reversed direction, but the hall hasn't
  for (int i = 195; i > 190; i--) {
    c.Update(i, true);
  }
  for (int i = 190; i > 185; i--) {
    c.Update(i, false);
  }
  ASSERT_FALSE(c.Calibrated());
  // The hall has reversed direction and is in sync with the main sensor
  for (int i = 185; i > 180; i--) {
    c.Update(i, true);
  }
  for (int i = 180; i < 200; i++) {
    c.Update(i, true);
  }
  c.Update(200, false);
  ASSERT_TRUE(c.Calibrated());
  ASSERT_NEAR(c.Update(150, true), 0, 1);
}

/*
 * SensorInaccuracies2 tests that when the hall sensor temporarily enters into
 * range of the magnet, calibration is not significantly affected
 */
TEST(HallCalibration, SensorInaccuracies2) {
  muan::HallCalibration c(0);
  for (int i = 0; i < 95; i++) {
    c.Update(i, false);
  }
  // The main sensor has reversed direction, but the hall hasn't
  for (int i = 95; i > 90; i--) {
    c.Update(i, false);
  }
  for (int i = 90; i > 85; i--) {
    c.Update(i, true);
  }
  // The hall has reversed direction and is in sync with the main sensor
  for (int i = 85; i > 0; i--) {
    c.Update(i, false);
  }
  // Now calibrate normally
  for (int i = 0; i < 100; i++) {
    c.Update(i, false);
  }
  for (int i = 100; i < 200; i++) {
    c.Update(i, true);
  }
  c.Update(200, false);
  ASSERT_TRUE(c.Calibrated());
  ASSERT_NEAR(c.Update(150, true), 0, 10);
}

/*
 * Test that even if the condition for being calibrated becomes false, it does
 * not become uncalibrated.
 */
TEST(HallCalibration, DoesntUncalibrate) {
  muan::HallCalibration c(0);
  // Calibrate normally
  for (int i = 0; i < 100; i++) {
    c.Update(i, false);
  }
  for (int i = 100; i < 200; i++) {
    c.Update(i, true);
  }
  for (int i = 200; i < 300; i++) {
    c.Update(i, false);
    // It currently meets the condition for being calibrated
    ASSERT_TRUE(c.Calibrated());
  }
  for (int i = 300; i < 400; i++) {
    c.Update(i, true);
    // It currently does not meet the condition for being calibrated, but it
    // was previously calibrated
    ASSERT_TRUE(c.Calibrated());
  }
  c.Update(400, false);
  // It now meets the condition for being calibrated
  ASSERT_TRUE(c.Calibrated());
}
