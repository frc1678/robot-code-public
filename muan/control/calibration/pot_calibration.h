#ifndef MUAN_CONTROL_CALIBRATION_POT_CALIBRATION_H_
#define MUAN_CONTROL_CALIBRATION_POT_CALIBRATION_H_

namespace muan {

namespace control {

class PotCalibration {
 public:
  PotCalibration(double units_per_index);
  ~PotCalibration();

  double Update(double enc_value, double pot_value, bool index_click);
  void Reset();

  // Returns if it's calibrated
  bool is_calibrated() const;

  // Returns true if there was an error in calibration after the initial
  // calibration (the index click is in the wrong place)
  bool index_error() const;

 private:
  double units_per_index_;

  // For averaging
  double offset_sum_;
  int average_counter_;

  int last_index_pulse_;
  bool has_index_pulse_;

  int offset_;
  bool calibrated_, index_error_;
};
}
}

#endif  // MUAN_CONTROL_CALIBRATION_POT_CALIBRATION_H_
