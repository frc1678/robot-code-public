#include "hall_calibration.h"
namespace muan {

HallCalibration::HallCalibration() :
  boundary_high_(0),
  boundary_low_(0),
  boundary_high_reached_(false),
  boundary_low_reached_(false),
  prev_main_value_(0),
  prev_hall_value_(false),
  first_time_(true) {}

HallCalibration::~HallCalibration() {}

void HallCalibration::Update(double main_value, bool hall_value) {
  // Moving into the range of the magnet
  // The first time this function is run, it will not have correct information
  // about the previous state
  if (!prev_hall_value_ && hall_value && !first_time_) {
    // High boundary reached by moving down into range
    // Don't change the measured high boundary on subsequent crosses
    if (prev_main_value_ > main_value && !boundary_high_reached_) {
      boundary_high_ = main_value;
      boundary_high_reached_ = true;
    }
    // Low boundary reached by moving up into range
    if (prev_main_value_ < main_value && !boundary_low_reached_) {
      boundary_low_ = main_value;
      boundary_low_reached_ = true;
    }
  }
  // Moving out of the range of the magnet
  if (prev_hall_value_ && !hall_value && !first_time_) {
    // High boundary reached by moving up out of range
    if (prev_main_value_ < main_value && !boundary_high_reached_) {
      boundary_high_ = main_value;
      boundary_high_reached_ = true;
    }
    // Low boundary reached by moving down out of range
    if (prev_main_value_ > main_value && !boundary_low_reached_) {
      boundary_low_ = main_value;
      boundary_low_reached_ = true;
    }
  }
  // Update values
  prev_main_value_ = main_value;
  prev_hall_value_ = hall_value;
  first_time_ = false;
  // Calibration is complete when both boundaries have been reached
  if (boundary_high_reached_ && boundary_low_reached_) {
    // Zero is at the center of the magnet's range
    offset_ = -(boundary_high_ + boundary_low_) / 2;
    calibrated_ = true;
  }
}

bool HallCalibration::Calibrated() { return calibrated_; }

double HallCalibration::Offset() { return offset_; }

} // namespace muan
