#ifndef MUAN_UTILS_OFFSET_PROFILED_CONTROLLER_POT_CALIBRATION_H_
#define MUAN_UTILS_OFFSET_PROFILED_CONTROLLER_POT_CALIBRATION_H_

namespace muan {

class PotCalibration {
 public:
  PotCalibration(int clicks_per_index, double clicks_per_pot,
                 double unit_per_click);
  ~PotCalibration();

  double Update(int enc_value, double pot_value, bool index_click);
  void Reset();

  bool is_calibrated() const;
  bool index_error() const;

 private:
  double clicks_per_index_, clicks_per_pot_, units_per_click_;

  // For averaging
  double offset_sum_;
  int average_counter_;

  int last_index_pulse_;
  bool has_index_pulse_;

  int offset_;
  bool calibrated_, index_error_;
};
}

#endif  // MUAN_UTILS_OFFSET_PROFILED_CONTROLLER_POT_CALIBRATION_H_
