#ifndef MUAN_HALL_CALIBRATION_H_
#define MUAN_HALL_CALIBRATION_H_

namespace muan {

/*
 * A class for calibrating a main sensor using a hall effect sensor
 */
class HallCalibration {
  public:
    HallCalibration(double magnet_position);
    ~HallCalibration();
    // Based on the raw sensor values, run calibration when calibration in not
    // complete, and return the offsetted sensor values if calibration is
    // complete. When calibration is not complete, the value returned is not
    // useful.
    double Update(double main_value, bool hall_value);
    bool Calibrated();
    double Offset();

  private:
    // The raw sensor values of the top and bottom of the magnet's range
    double boundary_high_;
    double boundary_low_;
    // Whether the high and low boundaries of the magnet have boon reached
    bool boundary_high_reached_;
    bool boundary_low_reached_;
    // The previous raw sensor values
    double prev_main_value_;
    bool prev_hall_value_;
    // Whether it is the first time running Update()
    bool first_time_;
    bool calibrated_;
    // The offset is the value *added* to the raw sensor values
    double offset_;
    // The value that should be returned at the center of the magnet
    double magnet_position_;
};
}

#endif // MUAN_HALL_CALIBRATION_H_
