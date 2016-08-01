#include "hall_calibration.h"
#include "gtest/gtest.h"

TEST(HallCalibration, Initializes) {
  muan::HallCalibration c;
  EXPECT_FALSE(c.Calibrated());
}

/* \ | |
 *  \| |
 *   \ |
 *   |\|
 *   | \
 *   | |\
 */
TEST(HallCalibration, CalibratesGoingUp) {
  muan::HallCalibration c;
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
  EXPECT_EQ(c.Offset(), -150);
}

/*   | | /
 *   | |/
 *   | /
 *   |/|
 *   / |
 *  /| |
 */
TEST(HallCalibration, CalibratesGoingDown) {
  muan::HallCalibration c;
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
  EXPECT_EQ(c.Offset(), -150);
}

/* \ | |
 *  \| |
 *   \ |
 *   |\|
 *   |/|
 *   / |
 *  /| |
 *  \| |
 *   \ |
 *   |\|
 *   | \
 *   | |\
 */
TEST(HallCalibration, ReverseFromOutside) {
  muan::HallCalibration c;
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
  EXPECT_EQ(c.Offset(), -150);
}

/*   |/|
 *   / |
 *  /| |
 *  \| |
 *   \ |
 *   |\|
 *   | \
 *   | |\
 */
TEST(HallCalibration, ReverseFromInside) {
  muan::HallCalibration c;
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
  EXPECT_EQ(c.Offset(), -150);
}
