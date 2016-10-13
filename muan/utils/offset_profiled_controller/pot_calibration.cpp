#include "pot_calibration.h"
#include <cmath>

namespace muan {

PotCalibration::PotCalibration(int clicks_per_index, double clicks_per_pot,
                               double units_per_click) {
  clicks_per_pot_ = clicks_per_pot;
  units_per_click_ = units_per_click;

  clicks_per_index_ = clicks_per_index;

  offset_ = 0;
  calibrated_ = false;
}

PotCalibration::~PotCalibration() {}

double PotCalibration::Update(int enc_value, double pot_value,
                              bool index_click) {
  if (index_click && !calibrated_ && average_counter_ >= 50) {
    double filtered_offset = enc_value + average_value_;
    int unoffset_value = filtered_offset - 0.5 * clicks_per_index_;
    int section = std::ceil(unoffset_value / clicks_per_index_);
    offset_ = -enc_value + section * clicks_per_index_;
    calibrated_ = true;
  }
  return (enc_value + offset_) * units_per_click_;
}

void PotCalibration::Reset() {
  calibrated_ = false;
  offset_ = 0;
}

const bool PotCalibration::is_calibrated() { return calibrated_; }

}  // namespace muan
