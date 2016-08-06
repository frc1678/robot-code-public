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
    // The max and min raw sensor values of when the hall sensor reads true, as
    // seen so far
    double max_hall_true_;
    double min_hall_true_;
    // The max and min raw sensor values that have been read
    double max_overall_;
    double min_overall_;
    // Whether it is the first time running Update()
    bool first_time_;
    // Whether the hall sensor has ever been triggered
    bool magnet_found_;
    bool calibrated_;
    // The offset is the value *added* to the raw sensor values
    double offset_;
    // The value that should be returned at the center of the magnet
    double magnet_position_;
};
}

#endif // MUAN_HALL_CALIBRATION_H_
