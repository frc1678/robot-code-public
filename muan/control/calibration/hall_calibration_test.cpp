#include "hall_calibration.h"
#include "gtest/gtest.h"

class HallCalibrationTest : public ::testing::Test {
 public:
  HallCalibrationTest() : calibration_(muan::control::HallCalibration(0)) {}
  // Update the calibration assuming sensors are at the same position
  void UpdateTest(double position) {
    // The magnet's range is 100 to 200 exclusive
    if (position > 100 && position < 200) {
      calibration_.Update(position, true);
    } else {
      calibration_.Update(position, false);
    }
  }
  // Update the calibration allowing for sensors being at different positions
  void UpdateTest(double main_sensor_position, double hall_sensor_position) {
    if (hall_sensor_position > 100 && hall_sensor_position < 200) {
      calibration_.Update(main_sensor_position, true);
    } else {
      calibration_.Update(main_sensor_position, false);
    }
  }
  bool is_calibrated() { return calibration_.is_calibrated(); }
  // Get the offsetted value from a raw sensor value
  double ValueAt(double position) { return calibration_.Update(position, false); }
  muan::control::HallCalibration calibration_;
};

/*
 * Make sure that it initializes to not calibrated
 */
TEST_F(HallCalibrationTest, Initializes) {
  calibration_ = muan::control::HallCalibration(0);
  EXPECT_FALSE(is_calibrated());
}

/*
 * Test that it calibrates going up, using all increasing values
 */
TEST_F(HallCalibrationTest, CalibratesGoingUp) {
  calibration_ = muan::control::HallCalibration(0);
  for (int i = 0; i < 200; i++) {
    UpdateTest(i);
    EXPECT_FALSE(is_calibrated());
  }
  UpdateTest(200);
  EXPECT_TRUE(is_calibrated());
}

/*
 * Test that it calibrates going down, using all decreasing values
 */
TEST_F(HallCalibrationTest, CalibratesGoingDown) {
  calibration_ = muan::control::HallCalibration(0);
  for (int i = 300; i > 100; i--) {
    UpdateTest(i);
    EXPECT_FALSE(is_calibrated());
  }
  UpdateTest(100);
  EXPECT_TRUE(is_calibrated());
}

/*
 * Starting from outside the magnet's range, test that going into the magnet's
 * range and then reversing back out does not calibrate it, and that going
 * through the magnet's range after that successfully calibrates it.
 */
TEST_F(HallCalibrationTest, ReverseFromOutside) {
  calibration_ = muan::control::HallCalibration(0);
  for (int i = 0; i < 150; i++) {
    UpdateTest(i);
    EXPECT_FALSE(is_calibrated());
  }
  for (int i = 150; i > 0; i--) {
    UpdateTest(i);
    EXPECT_FALSE(is_calibrated());
  }
  for (int i = 0; i < 200; i++) {
    UpdateTest(i);
    EXPECT_FALSE(is_calibrated());
  }
  UpdateTest(200);
  EXPECT_TRUE(is_calibrated());
}

/*
 * Test that starting in the magnet's range and moving out does not calibrate
 * it, and that going through the magnet's range after that successfully
 * calibrates it.
 */
TEST_F(HallCalibrationTest, ReverseFromInside) {
  calibration_ = muan::control::HallCalibration(0);
  for (int i = 150; i > 0; i--) {
    UpdateTest(i);
    EXPECT_FALSE(is_calibrated());
  }
  for (int i = 0; i < 200; i++) {
    UpdateTest(i);
    EXPECT_FALSE(is_calibrated());
  }
  UpdateTest(200);
  EXPECT_TRUE(is_calibrated());
}

/*
 * Test that the calibration routine finds the center of the magnet's range
 * under normal conditions.
 */
TEST_F(HallCalibrationTest, FindsMagnetCenter) {
  calibration_ = muan::control::HallCalibration(0);
  for (int i = 0; i < 200; i++) {
    UpdateTest(i);
  }
  UpdateTest(200);
  ASSERT_TRUE(is_calibrated());
  EXPECT_NEAR(ValueAt(150), 0, 1);
}

/*
 * Test that a nonzero value can be used as the center of the magnet
 */
TEST_F(HallCalibrationTest, UsesMagnetPosition) {
  calibration_ = muan::control::HallCalibration(1000);
  for (int i = 0; i < 200; i++) {
    UpdateTest(i);
  }
  UpdateTest(200);
  ASSERT_TRUE(is_calibrated());
  EXPECT_NEAR(ValueAt(150), 1000, 1);
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
TEST_F(HallCalibrationTest, SensorInaccuracies1) {
  calibration_ = muan::control::HallCalibration(0);
  for (int i = 0; i < 195; i++) {
    UpdateTest(i, i);
  }
  // The main sensor has reversed direction, but the hall hasn't
  for (int i = 0; i < 10; i++) {
    UpdateTest(195 - i, 195 + i);
  }
  ASSERT_FALSE(is_calibrated());
  // The hall has reversed direction and is in sync with the main sensor
  for (int i = 185; i > 180; i--) {
    UpdateTest(i, i);
  }
  for (int i = 180; i < 200; i++) {
    UpdateTest(i, i);
  }
  UpdateTest(200, 200);
  ASSERT_TRUE(is_calibrated());
  ASSERT_NEAR(ValueAt(150), 0, 1);
}

/*
 * SensorInaccuracies2 tests that when the hall sensor temporarily enters into
 * range of the magnet, calibration is not significantly affected
 */
TEST_F(HallCalibrationTest, SensorInaccuracies2) {
  calibration_ = muan::control::HallCalibration(0);
  for (int i = 0; i < 95; i++) {
    UpdateTest(i, i);
  }
  // The main sensor has reversed direction, but the hall hasn't
  for (int i = 0; i < 10; i++) {
    UpdateTest(95 - i, 95 + i);
  }
  // The hall has reversed direction and is in sync with the main sensor
  for (int i = 85; i > 0; i--) {
    UpdateTest(i, i);
  }
  // Now calibrate normally
  for (int i = 0; i < 200; i++) {
    UpdateTest(i, i);
  }
  UpdateTest(200, 200);
  ASSERT_TRUE(is_calibrated());
  ASSERT_NEAR(ValueAt(150), 0, 10);
}

/*
 * Test that even if the condition for being calibrated becomes false, it does
 * not become uncalibrated.
 */
TEST_F(HallCalibrationTest, DoesntUncalibrate) {
  muan::control::HallCalibration calibration(0);
  // Calibrate normally
  // Don't use the fixture because this doesn't really follow any model
  for (int i = 0; i < 100; i++) {
    calibration.Update(i, false);
  }
  for (int i = 100; i < 200; i++) {
    calibration.Update(i, true);
  }
  for (int i = 200; i < 300; i++) {
    calibration.Update(i, false);
    // It currently meets the condition for being calibrated
    ASSERT_TRUE(calibration.is_calibrated());
  }
  for (int i = 300; i < 400; i++) {
    calibration.Update(i, true);
    // It currently does not meet the condition for being calibrated, but it
    // was previously calibrated
    ASSERT_TRUE(calibration.is_calibrated());
  }
  calibration.Update(400, false);
  // It now meets the condition for being calibrated
  ASSERT_TRUE(calibration.is_calibrated());
}
