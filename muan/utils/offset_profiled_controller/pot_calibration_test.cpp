#include "pot_calibration.h"
#include "muan/utils/math_utils.h"
#include "gtest/gtest.h" #include < iostream >

class PotCalibrationTest : public ::testing::Test {
 public:
  PotCalibrationTest() : calibration_(10) { calibrated_value_ = 0; }

  void UpdateTest(int initial_pot_value, int ending_difference,
                  int noise_range) {
    bool index_click = false;
    pot_value_ = initial_pot_value;

    // The logic needs to be different if the motion is in the positive or
    // negative direction.
    if (ending_difference > 0) {
      // Simulate a potentiometer and encoder moving in the positive direction.
      for (int uncalibrated_value = 0; uncalibrated_value <= ending_difference;
           uncalibrated_value++, pot_value_++) {
        // Check if it should be an index click. This isn't technically correct,
        // but since the potentiometer value is as accurate as the encoder
        // value, this works just as well.
        if (pot_value_ % 10 == 0) {
          index_click = true;
        } else {
          index_click = false;
        }

        // Add noise to the potentiometer for the calibration.
        double noisy_pot_value =
            pot_value_ + muan::GaussianNoise(noise_range, 0);

        // Get the calibrated value.
        calibrated_value_ = calibration_.Update(uncalibrated_value,
                                                noisy_pot_value, index_click);
      }

      // This is neccesary because for some reason it incremented one too many
      // times.
      pot_value_--;
    } else {
      // Simulate a potentiometer and encoder moving in the negative direction.
      for (int uncalibrated_value = 0; uncalibrated_value >= ending_difference;
           uncalibrated_value--, pot_value_--) {
        // Check if it should be an index click. This isn't technically correct,
        // but since the potentiometer value is as accurate as the encoder
        // value, this works just as well.
        if (pot_value_ % 10 == 0) {
          index_click = true;
        } else {
          index_click = false;
        }

        // Add noise to the potentiometer for the calibration.
        double noisy_pot_value =
            pot_value_ + muan::GaussianNoise(noise_range, 0);
        calibrated_value_ = calibration_.Update(uncalibrated_value,
                                                noisy_pot_value, index_click);
      }

      // This is neccesary because for some reason it incremented one too few
      // times.
      pot_value_++;
    }
  }

  bool is_calibrated() { return calibration_.is_calibrated(); }

  std::tuple<int, int> get_final_values() {
    return std::make_tuple(calibrated_value_, pot_value_);
  }

  void ResetTest() { calibration_.Reset(); }

  muan::PotCalibration calibration_;
  int calibrated_value_, pot_value_;
};

// This test tests all cases between 0 and any given range of movement and
// potentiometer starting position.
TEST_F(PotCalibrationTest, UniversalCases) {
  int range = 200;  // Edit this to test fewer or more cases.

  // Cycle through tons and tons of initial values, distance it moves, and the
  // noise of the potentiometer.
  for (int pot_value = -range / 2; pot_value <= range / 2; pot_value++) {
    for (int ending_difference = -range / 2; ending_difference <= range / 2;
         ending_difference++) {
      for (int noise_range = 0; noise_range < 2; noise_range++) {
        UpdateTest(pot_value, ending_difference, noise_range);

        // Logic for figuring out if the resulting movement from the intial
        // value is going to result in getting calibrated.
        if (std::floor(double(pot_value) / 10.0) !=
                std::floor(double(pot_value + ending_difference) / 10.0) ||
            muan::true_modulo(pot_value + ending_difference, 10) == 0 ||
            muan::true_modulo(pot_value, 10) == 0) {
          EXPECT_TRUE(is_calibrated());
          EXPECT_EQ(std::get<0>(get_final_values()),
                    std::get<1>(get_final_values()));
        } else {
          EXPECT_FALSE(is_calibrated());
        }

        ResetTest();
      }
    }
  }
}
