#include "muan/control/calibration/pot_calibration.h"
#include <algorithm>
#include <tuple>
#include "gtest/gtest.h"
#include "muan/utils/math_utils.h"

class PotCalibrationTest : public ::testing::Test {
 public:
  PotCalibrationTest() : calibration_(10.0) { calibrated_value_ = 0; }

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
            initial_system_value + muan::utils::GaussianNoise(noise_range, 0);
        calibrated_value_ = calibration_.Update(0, pot_value, false);
      }

      // Simulate a potentiometer and encoder moving in the positive direction.
      for (int uncalibrated_value = 0; uncalibrated_value <= ending_difference;
           uncalibrated_value++, system_value_++) {
        // Check if it should be an index click
        if (static_cast<int>(system_value_) % 10 == 0) {
          index_click = true;
        } else {
          index_click = false;
        }

        // Add noise to the potentiometer for the calibration.
        double pot_value =
            system_value_ + muan::utils::GaussianNoise(noise_range, 0);

        // Get the calibrated value.
        calibrated_value_ =
            calibration_.Update(uncalibrated_value, pot_value, index_click);
      }

      for (int i = 0; i < 50; i++) {
        double pot_value = initial_system_value + ending_difference +
                           muan::utils::GaussianNoise(noise_range, 0);
        calibrated_value_ =
            calibration_.Update(ending_difference, pot_value, index_click);
      }
      // This is neccesary because the loop increments after the updating of the
      // calibration object and therefore is one too many.
      system_value_--;
    } else {
      // Have the system wait a little bit when the movement is too little,
      // allowing the calibration to accumulate a better average
      for (int i = 0; i < std::max(0, 50 + ending_difference); i++) {
        double pot_value =
            initial_system_value + muan::utils::GaussianNoise(noise_range, 0);
        calibrated_value_ = calibration_.Update(0, pot_value, false);
      }
      // Simulate a potentiometer and encoder moving in the negative
      // direction.
      for (int uncalibrated_value = 0; uncalibrated_value >= ending_difference;
           uncalibrated_value--, system_value_--) {
        // Checks if it should be an index click
        if (static_cast<int>(system_value_) % 10 == 0) {
          index_click = true;
        } else {
          index_click = false;
        }

        // Add noise to the potentiometer for the calibration.
        double pot_value =
            system_value_ + muan::utils::GaussianNoise(noise_range, 0);
        calibrated_value_ =
            calibration_.Update(uncalibrated_value, pot_value, index_click);
      }

      for (int i = 0; i < 50; i++) {
        double pot_value = initial_system_value + ending_difference +
                           muan::utils::GaussianNoise(noise_range, 0);
        calibrated_value_ =
            calibration_.Update(ending_difference, pot_value, index_click);
      }

      // This is neccesary because the loop increments after the updating of the
      // calibration object and therefore is one too little.
      system_value_++;
    }
  }

  bool is_calibrated() { return calibration_.is_calibrated(); }
  bool get_index_error() { return calibration_.index_error(); }

  std::tuple<double, double> get_final_values() {
    return std::make_tuple(calibrated_value_, system_value_);
  }

  void ResetTest() { calibration_.Reset(); }

  muan::control::PotCalibration calibration_;
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
        // value is going to result in getting calibrated. If the system is in
        // the same section of index clicks at the beginning and the end, then
        // it shouldn't be calibrated. Alternatively, if the system value is
        // calibrated at the beginning or the end of the iteration, it should be
        // considered calibrated.
        if (std::floor(system_value / 10.0) !=
                std::floor((system_value + ending_difference) / 10.0) ||
            muan::utils::true_modulo(
                static_cast<int>(system_value + ending_difference), 10) == 0 ||
            muan::utils::true_modulo(static_cast<int>(system_value), 10) == 0) {
          EXPECT_TRUE(is_calibrated());
          EXPECT_NEAR(std::get<0>(get_final_values()),
                      std::get<1>(get_final_values()), 1e-5);
        } else {
          EXPECT_FALSE(is_calibrated());
        }
        EXPECT_FALSE(get_index_error());
        ResetTest();
      }
    }
  }
}

TEST_F(PotCalibrationTest, CalibrationError) {
  muan::control::PotCalibration calibration_error(10);
  double system_value = 19;
  int uncalibrated_value;
  bool index_click = false;

  for (uncalibrated_value = 0; uncalibrated_value <= 60;
       uncalibrated_value++, system_value++) {
    // Check if it should be an index click
    if (static_cast<int>(system_value) % 10 == 0) {
      index_click = true;
    } else {
      index_click = false;
    }

    // Add noise to the potentiometer for the calibration.
    double pot_value = system_value + muan::utils::GaussianNoise(5, 0);

    // Get the calibrated value.
    calibration_error.Update(uncalibrated_value, pot_value, index_click);
  }

  for (; uncalibrated_value <= 150; uncalibrated_value++, system_value++) {
    if (static_cast<int>(system_value) % 10 == 0) {
      index_click = true;
    } else {
      index_click = false;
    }

    // By adding a sudden offset that changes the index click's frame of
    // reference, it should recognize the error.
    double pot_value = system_value + 15 + muan::utils::GaussianNoise(5, 0);

    calibration_error.Update(uncalibrated_value, pot_value, index_click);
  }

  EXPECT_TRUE(calibration_error.is_calibrated());
  EXPECT_TRUE(calibration_error.index_error());
}
