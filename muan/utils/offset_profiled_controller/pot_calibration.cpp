#include "pot_calibration.h"
#include <cmath>

namespace muan {

PotCalibration::PotCalibration(double units_per_index) {
  units_per_index_ = units_per_index;

  offset_ = 0;
  has_index_pulse_ = false;
  calibrated_ = false;
  index_error_ = false;
  offset_sum_ = 0;
  average_counter_ = 0;
}

PotCalibration::~PotCalibration() {}

double PotCalibration::Update(double enc_value, double pot_value,
                              bool index_click) {
  // Makes an average of the offset from the encoder and the potentiometer,
  // hopefully taking care of the poteniometer noise
  offset_sum_ += (pot_value - enc_value);
  average_counter_++;
  double average_offset = offset_sum_ / average_counter_;

  if (index_click) {
    has_index_pulse_ = true;
    last_index_pulse_ = enc_value;
  }

  // Only calibrate if the average has had time to accumulate a good offset
  if (has_index_pulse_ && average_counter_ >= 50) {
    double filtered_offset = last_index_pulse_ + average_offset;
    double unoffset_value = filtered_offset - 0.5 * units_per_index_;
    int section = std::ceil(unoffset_value / units_per_index_);

    // Only runs if it hasn't calibrated before, then sets the offset to the
    // newly calibrated offset
    if (!calibrated_) {
      offset_ = -last_index_pulse_ + section * units_per_index_;
      calibrated_ = true;

      // Error checking, changes a boolean if there is a change in offset
    } else if (offset_ != -last_index_pulse_ + section * units_per_index_) {
      index_error_ = true;
    }
    has_index_pulse_ = false;
  }
  return (enc_value + offset_);
}

void PotCalibration::Reset() {
  calibrated_ = false;
  has_index_pulse_ = false;
  index_error_ = false;
  offset_ = 0;
  offset_sum_ = 0;
  average_counter_ = 0;
}

bool PotCalibration::is_calibrated() const { return calibrated_; }

bool PotCalibration::index_error() const { return index_error_; }

}  // namespace muan
