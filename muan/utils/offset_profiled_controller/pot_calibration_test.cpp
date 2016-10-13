#include "pot_calibration.h"
#include "muan/utils/math_utils.h"
#include "gtest/gtest.h"

class PotCalibrationTest : public ::testing::Test {
 public:
  PotCalibrationTest() : calibration_(10, 1, 1) { calibrated_value_ = 0; }

  void UpdateTest(int initial_system_value, int ending_difference,
                  int noise_range) {
    bool index_click = false;
    system_value_ = initial_system_value;

    // The logic needs to be different if the motion is in the positive or
    // negative direction.
    if (ending_difference > 0) {
      // Have the system wait a little bit when the system's movement is too
      // little to allow it to get a better average
      for (int i = 0; i < std::max(0, 5 - ending_difference); i++) {
        double pot_value =
            initial_system_value + muan::GaussianNoise(noise_range, 0);
        calibrated_value_ = calibration_.Update(0, pot_value, false);
      }

      // Simulate a potentiometer and encoder moving in the positive direction.
      for (int uncalibrated_value = 0; uncalibrated_value <= ending_difference;
           uncalibrated_value++, system_value_++) {
        // Check if it should be an index click
        if (int(system_value_) % 10 == 0) {
          index_click = true;
        } else {
          index_click = false;
        }

        // Add noise to the potentiometer for the calibration.
        double pot_value = system_value_ + muan::GaussianNoise(noise_range, 0);

        // Get the calibrated value.
        calibrated_value_ =
            calibration_.Update(uncalibrated_value, pot_value, index_click);
      }

      for (int i = 0; i < 50; i++) {
        double pot_value = initial_system_value + ending_difference +
                           muan::GaussianNoise(noise_range, 0);
        calibrated_value_ =
            calibration_.Update(ending_difference, pot_value, index_click);
      }
      // This is neccesary because for some reason it incremented one too many
      // times.
      system_value_--;
    } else {
      // Have the system wait a little bit when the movement is too little,
      // allowing the calibration to accumulate a better average
      for (int i = 0; i < std::max(0, 50 + ending_difference); i++) {
        double pot_value =
            initial_system_value + muan::GaussianNoise(noise_range, 0);
        calibrated_value_ = calibration_.Update(0, pot_value, false);
      }
      // Simulate a potentiometer and encoder moving in the negative
      // direction.
      for (int uncalibrated_value = 0; uncalibrated_value >= ending_difference;
           uncalibrated_value--, system_value_--) {
        // Checks if it should be an index click
        if (int(system_value_) % 10 == 0) {
          index_click = true;
        } else {
          index_click = false;
        }

        // Add noise to the potentiometer for the calibration.
        double pot_value = system_value_ + muan::GaussianNoise(noise_range, 0);
        calibrated_value_ =
            calibration_.Update(uncalibrated_value, pot_value, index_click);
      }

      for (int i = 0; i < 50; i++) {
        double pot_value = initial_system_value + ending_difference +
                           muan::GaussianNoise(noise_range, 0);
        calibrated_value_ =
            calibration_.Update(ending_difference, pot_value, index_click);
      }

      // This is neccesary because for some reason it incremented one too few
      // times.
      system_value_++;
    }
  }

  bool is_calibrated() { return calibration_.is_calibrated(); }

  std::tuple<double, double> get_final_values() {
    return std::make_tuple(calibrated_value_, system_value_);
  }

  double get_average_value() { return calibration_.get_average_value(); }

  void ResetTest() { calibration_.Reset(); }

  muan::PotCalibration calibration_;
  double system_value_, calibrated_value_;
};

// This test tests all cases between 0 and any given range of movement and
// potentiometer starting position.
TEST_F(PotCalibrationTest, UniversalCases) {
  int range = 200;  // Edit this to test fewer or more cases.

  // Cycle through tons and tons of initial values, distance it moves, and the
  // noise of the potentiometer.
  for (double system_value = -range / 2; system_value <= range / 2;
       system_value += 3) {
    for (double ending_difference = -range / 2; ending_difference <= range / 2;
         ending_difference += 3) {
      for (int noise_range = 0; noise_range < 5; noise_range++) {
        UpdateTest(system_value, ending_difference, noise_range);

        // Logic for figuring out if the resulting movement from the initial
        // value is going to result in getting calibrated.
        if (std::floor(system_value / 10.0) !=
                std::floor((system_value + ending_difference) / 10.0) ||
            muan::true_modulo(int(system_value + ending_difference), 10) == 0 ||
            muan::true_modulo(int(system_value), 10) == 0) {
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

TEST_F(PotCalibrationTest, Scaling) {
  // This is establishing the scaling factors for the code. 10 is the amount
  // of encoder clicks per one index click, 0.5 is the amount of encoder clicks
  // per one potentiometer "click", and 0.25 is the amount of encoder clicks per
  // system "clicks" (the scaling factor for going from encoder clicks to the
  // final unit output)
  muan::PotCalibration calibration_scaled(10, 0.5, 0.25);
  int uncalibrated_value = 0;
  bool index_click = false;
  for (double system_value = 12; system_value <= 24;
       system_value += 0.25, uncalibrated_value++) {
    if (uncalibrated_value % 10 == 0) {
      index_click = true;
    } else {
      index_click = false;
    }
    double pot_value = system_value * 2.0 + muan::GaussianNoise(1, 0);
    calibration_scaled.Update(uncalibrated_value, pot_value, index_click);
  }
}
