#ifndef MUAN_HALL_CALIBRATION_H_
#define MUAN_HALL_CALIBRATION_H_

namespace muan {

/*
 * A class for calibrating a main sensor using a hall effect sensor
 */
class HallCalibration {
  public:
    HallCalibration();
    ~HallCalibration();
    void Update(double main_value, bool hall_value);
    bool Calibrated();
    double Offset();

  private:
    double boundary_high_;
    double boundary_low_;
    bool boundary_high_reached_;
    bool boundary_low_reached_;
    double prev_main_value_;
    bool prev_hall_value_;
    bool first_time_;
    bool calibrated_;
    double offset_;
};
}

#endif // MUAN_HALL_CALIBRATION_H_
