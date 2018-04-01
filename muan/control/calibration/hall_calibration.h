#ifndef MUAN_CONTROL_CALIBRATION_HALL_CALIBRATION_H_
#define MUAN_CONTROL_CALIBRATION_HALL_CALIBRATION_H_

namespace muan {
namespace control {

/*
 * A class for calibrating a main sensor using a hall effect sensor. To use
 * call Update in a loop.
 *
 * Example:
 *   double angle = calibration.Calibrate(encoder_value, hall_value);
 *   if (calibration.is_calibrated())
 *     DoSomething(angle);
 *   else
 *     state = still_calibrating;
 */
class HallCalibration {
 public:
  explicit HallCalibration(double magnet_position);
  ~HallCalibration() = default;
  // Based on the raw sensor values, run calibration when calibration in not
  // complete, and return the offsetted sensor values if calibration is
  // complete. When calibration is not complete, the value returned is not
  // useful.
  double Update(double main_sensor_value, bool hall_value);
  bool is_calibrated() const;
  double offset() const;

 private:
  // The max and min raw sensor values of when the hall sensor reads true, as
  // seen so far. When calibration is complete it is approximately the top
  // and bottom of the magnet's range.
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

}  // namespace control
}  // namespace muan

#endif  // MUAN_CONTROL_CALIBRATION_HALL_CALIBRATION_H_
