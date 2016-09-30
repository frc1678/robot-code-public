#include "pot_calibration.h"
#include <cmath>

namespace muan {

PotCalibration::PotCalibration(int clicks_per_index) {
  clicks_per_index_ = clicks_per_index;
  offset_ = 0;
  calibrated_ = false;
}

PotCalibration::~PotCalibration() {}

int PotCalibration::Update(int enc_value, double pot_value, bool index_click) {
  if (index_click) {
    int unoffset_value = pot_value - 0.5 * clicks_per_index_;
    int section = std::ceil(unoffset_value / clicks_per_index_);
    offset_ = -enc_value + section * clicks_per_index_;
    calibrated_ = true;
  }
  return enc_value + offset_;
}

void PotCalibration::Reset() {
  calibrated_ = false;
  offset_ = 0;
}

const bool PotCalibration::is_calibrated() { return calibrated_; }

}  // namespace muan
