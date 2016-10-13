#include "pot_calibration.h"
#include <cmath>

namespace muan {

PotCalibration::PotCalibration(int clicks_per_index, double clicks_per_pot,
                               double units_per_click) {
  clicks_per_pot_ = clicks_per_pot;
  units_per_click_ = units_per_click;

  clicks_per_index_ = clicks_per_index;

  offset_ = 0;
  early_calibration_ = false;
  calibrated_ = false;
  average_value_ = 0;
  average_counter_ = 0;
}

PotCalibration::~PotCalibration() {}

double PotCalibration::Update(int enc_value, double pot_value,
                              bool index_click) {
  // Makes an average of the offset from the encoder and the potentiometer,
  // hopefully taking care of the potentiometer noise
  average_value_ = ((average_counter_ * average_value_) +
                    ((pot_value * clicks_per_pot_) - enc_value)) /
                   (average_counter_ + 1);
  average_counter_++;

  // Gets a more accurate reading off of the average to use in the calibration
  if (index_click && !calibrated_ && average_counter_ >= 50) {
    double filtered_offset = enc_value + average_value_;
    int unoffset_value = filtered_offset - 0.5 * clicks_per_index_;
    int section = std::ceil(unoffset_value / clicks_per_index_);
    offset_ = -enc_value + section * clicks_per_index_;
    calibrated_ = true;
  }

  if (index_click && !calibrated_ && average_counter_ < 50) {
    logged_enc_value_ = enc_value;
    early_calibration_ = true;
  }

  if (!calibrated_ && early_calibration_ && average_counter_ >= 50) {
    double filtered_offset = logged_enc_value_ + average_value_;
    int unoffset_value = filtered_offset - 0.5 * clicks_per_index_;
    int section = std::ceil(unoffset_value / clicks_per_index_);
    offset_ = -logged_enc_value_ + section * clicks_per_index_;
    calibrated_ = true;
  }
  return (enc_value + offset_) * units_per_click_;
}

void PotCalibration::Reset() {
  calibrated_ = false;
  early_calibration_ = false;
  offset_ = 0;
  average_value_ = 0;
  average_counter_ = 0;
}

double PotCalibration::get_average_value() const { return average_value_; }

bool PotCalibration::is_calibrated() const { return calibrated_; }

}  // namespace muan
