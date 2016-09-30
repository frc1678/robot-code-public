#ifndef MUAN_POT_CALIBRATION_H_
#define MUAN_POT_CALIBRATION_H_

namespace muan {

class PotCalibration {
 public:
  PotCalibration(int clicks_per_index);
  ~PotCalibration();

  int Update(int enc_value, double pot_value, bool index_click);
  void Reset();

  const bool is_calibrated();

 private:
  double clicks_per_index_;

  int offset_;

  bool calibrated_;
};
}

#endif  // MUAN_POT_CALIBRATION_H_
