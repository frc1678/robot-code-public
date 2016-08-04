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
  for(int i = 0; i < 100; i++) {
    c.Update(i, false);
    EXPECT_FALSE(c.Calibrated());
  }
  for(int i = 100; i < 200; i++) {
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
  for(int i = 300; i > 200; i--) {
    c.Update(i, false);
    EXPECT_FALSE(c.Calibrated());
  }
  for(int i = 200; i > 100; i--) {
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
  for(int i = 0; i < 100; i++) {
    c.Update(i, false);
    EXPECT_FALSE(c.Calibrated());
  }
  for(int i = 100; i < 150; i++) {
    c.Update(i, true);
    EXPECT_FALSE(c.Calibrated());
  }
  for(int i = 150; i > 100; i--) {
    c.Update(i, true);
    EXPECT_FALSE(c.Calibrated());
  }
  for(int i = 100; i > 0; i--) {
    c.Update(i, false);
    EXPECT_FALSE(c.Calibrated());
  }
  for(int i = 0; i < 100; i++) {
    c.Update(i, false);
    EXPECT_FALSE(c.Calibrated());
  }
  for(int i = 100; i < 200; i++) {
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
  for(int i = 150; i > 100; i--) {
    c.Update(i, true);
    EXPECT_FALSE(c.Calibrated());
  }
  for(int i = 100; i > 0; i--) {
    c.Update(i, false);
    EXPECT_FALSE(c.Calibrated());
  }
  for(int i = 0; i < 100; i++) {
    c.Update(i, false);
    EXPECT_FALSE(c.Calibrated());
  }
  for(int i = 100; i < 200; i++) {
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
  for(int i = 0; i < 100; i++) {
    c.Update(i, false);
  }
  for(int i = 100; i < 200; i++) {
    c.Update(i, true);
  }
  c.Update(200, false);
  EXPECT_EQ(c.Update(150, true), 0);
}

/*
 * Test that a nonzero value can be used as the center of the magnet
 */
TEST(HallCalibration, UsesMagnetPosition) {
  muan::HallCalibration c(1000);
  for(int i = 0; i < 100; i++) {
    c.Update(i, false);
  }
  for(int i = 100; i < 200; i++) {
    c.Update(i, true);
  }
  c.Update(200, false);
  EXPECT_EQ(c.Update(150, true), 1000);
}
