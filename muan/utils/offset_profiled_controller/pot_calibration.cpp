#include "pot_calibration.h"
#include <cmath>
#include <iostream>

namespace muan {

PotCalibration::PotCalibration(int clicks_per_index, double clicks_per_pot,
                               double units_per_click) {
  clicks_per_pot_ = clicks_per_pot;
  units_per_click_ = units_per_click;

  clicks_per_index_ = clicks_per_index;

  offset_ = 0;
  has_index_pulse_ = false;
  calibrated_ = false;
  index_error_ = false;
  offset_sum_ = 0;
  average_counter_ = 0;
}

PotCalibration::~PotCalibration() {}

double PotCalibration::Update(int enc_value, double pot_value,
                              bool index_click) {
  // Makes an average of the offset from the encoder and the potentiometer,
  // hopefully taking care of the potentiometer noise
  offset_sum_ += (pot_value * clicks_per_pot_ - enc_value);
  average_counter_++;
  double average_offset = offset_sum_ / average_counter_;

  if (index_click) {
    has_index_pulse_ = true;
    last_index_pulse_ = enc_value;
  }

  // Gets a more accurate reading off of the average to use in the calibration
  if (has_index_pulse_ && average_counter_ >= 50) {
    double filtered_offset = last_index_pulse_ + average_offset;
    int unoffset_value = filtered_offset - 0.5 * clicks_per_index_;
    int section = std::ceil(unoffset_value / clicks_per_index_);
    if (!calibrated_) {
      offset_ = -last_index_pulse_ + section * clicks_per_index_;
      calibrated_ = true;
    } else if (offset_ != -last_index_pulse_ + section * clicks_per_index_) {
      std::cout << offset_ << " "
                << -last_index_pulse_ + section * clicks_per_index_
                << std::endl;
      index_error_ = true;
    }
  }
  return (enc_value + offset_) * units_per_click_;
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
