#include "hall_encoder_calibration.h"
#include "gtest/gtest.h"

TEST(HallEncoderCalibration, Initializes) {
  muan::Calibration<int, bool> *c = new muan::HallEncoderCalibration();
  EXPECT_FALSE(c->Calibrated());
}

/* \ | |
 *  \| |
 *   \ |
 *   |\|
 *   | \
 *   | |\
 */
TEST(HallEncoderCalibration, CalibratesGoingUp) {
  muan::HallEncoderCalibration c = muan::HallEncoderCalibration();
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
TEST(HallEncoderCalibration, CalibratesGoingDown) {
  muan::HallEncoderCalibration c = muan::HallEncoderCalibration();
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
TEST(HallEncoderCalibration, ReverseFromOutside) {
  muan::HallEncoderCalibration c = muan::HallEncoderCalibration();
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
TEST(HallEncoderCalibration, ReverseFromInside) {
  muan::HallEncoderCalibration c = muan::HallEncoderCalibration();
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
