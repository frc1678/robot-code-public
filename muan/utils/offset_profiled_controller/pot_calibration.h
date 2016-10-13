#ifndef MUAN_POT_CALIBRATION_H_
#define MUAN_POT_CALIBRATION_H_

namespace muan {

class PotCalibration {
 public:
  PotCalibration(int clicks_per_index, double clicks_per_pot,
                 double unit_per_click);
  ~PotCalibration();

  double Update(int enc_value, double pot_value, bool index_click);
  void Reset();

  bool is_calibrated() const;
  double get_average_value() const;

 private:
  double clicks_per_index_, clicks_per_pot_, units_per_click_, average_value_;

  int offset_, average_counter_, logged_enc_value_;
  bool calibrated_, early_calibration_;
};
}

#endif  // MUAN_POT_CALIBRATION_H_
