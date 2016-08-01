#ifndef MUAN_HALL_ENCODER_CALIBRATION_H_
#define MUAN_HALL_ENCODER_CALIBRATION_H_

namespace muan {

/*
 * A class for calibrating an encoder using a hall effect sensor
 */
class HallEncoderCalibration {
  public:
    HallEncoderCalibration();
    ~HallEncoderCalibration();
    void Update(int encoder_value, bool hall_value);
    bool Calibrated();
    int Offset();

  private:
    int boundary_high_;
    int boundary_low_;
    bool boundary_high_reached_;
    bool boundary_low_reached_;
    int prev_encoder_value_;
    bool prev_hall_value_;
    bool first_time_;
    bool calibrated_;
    int offset_;
};
}

#endif // MUAN_HALL_ENCODER_CALIBRATION_H_
