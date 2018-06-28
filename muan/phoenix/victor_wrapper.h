#ifndef MUAN_PHOENIX_VICTOR_WRAPPER_H_
#define MUAN_PHOENIX_VICTOR_WRAPPER_H_

#include "ctre/Phoenix.h"
#include "muan/units/units.h"

namespace muan {
namespace phoenix {

constexpr int kVictorSetupTimeout = 100;
constexpr int kVictorRegularTimeout = 10;
constexpr int kVictorOutput = 1023. / 12.;  // Per volt

using muan::units::ms;

class VictorWrapper {
 public:
  enum class FeedbackSensor {
    kMagEncoderRelative,
    kMagEncoderAbsolute,
    kNone,
  };

  struct Gains {  // SI Units, mechanism specific
    double p;
    double i;
    double d;
    double f;
    double i_zone;
    double max_integral;
    double deadband;
  };

  struct Config {
    // Neutral
    NeutralMode neutral_mode = NeutralMode::Coast;
    double neutral_deadband = 0.04;

    // Limits
    bool enable_current_limit = false;
    bool enable_soft_limit = false;
    bool enable_limit_switch = false;
    int forward_soft_limit = 0;
    int reverse_soft_limit = 0;

    bool motor_inverted = false;
    bool sensor_inverted = false;

    // Frame periods (ms)
    int control_frame_period = 5;
    int motion_control_frame_period = 100;
    int general_status_frame_rate = 5;
    int feedback_status_frame_rate = 5;
    int quadrature_status_frame_rate = 5;
    int analog_temp_vbat_status_frame_rate = 5;
    int pwm_status_frame_rate = 5;

    // Velocity Measurement
    VelocityMeasPeriod velocity_measurement_period =
        VelocityMeasPeriod::Period_100Ms;
    int velocity_measurement_window = 32;  // samples / period

    // Ramp rates
    double open_loop_ramp_time = 0.;    // seconds to 12V
    double closed_loop_ramp_time = 0.;  // seconds to 12V

    double conversion_factor = 1.;  // native / actual
    FeedbackSensor sensor = FeedbackSensor::kNone;

    bool compensate_voltage = true;
    double max_voltage = 12.;
  };

  VictorWrapper(int id, Config config);  // Specified config
  void LoadConfig(Config config);

  // Set victor output
  void SetFollower(int id) { victor_.Set(ControlMode::Follower, id); }
  void SetOpenloopGoal(double setpoint);
  void SetPositionGoal(double setpoint, double setpoint_ff);
  void SetVelocityGoal(double setpoint, double setpoint_ff);

  // PID gains for a given slot
  void SetGains(Gains gains, int slot);
  void SelectGains(int slot);

  void ResetSensor(double value = 0) {
    victor_.SetSelectedSensorPosition(value, 0, kVictorRegularTimeout);
  }

  // Getters
  inline int id() { return victor_.GetDeviceID(); }
  inline Config config() { return config_; }
  inline double position() {
    return victor_.GetSelectedSensorPosition(0) / conversion_factor_;
  }
  inline double velocity() {
    return victor_.GetSelectedSensorVelocity(0) /
           (conversion_factor_ * 100 * ms);
  }

  inline double voltage() { return victor_.GetMotorOutputVoltage(); }
  inline double percent() { return victor_.GetMotorOutputPercent(); }
  inline double current() { return victor_.GetOutputCurrent(); }
  inline VictorSPX* victor() { return &victor_; }

 protected:
  VictorSPX victor_;

 private:
  Config config_;

  double conversion_factor_;  // native / actual
};

}  // namespace phoenix
}  // namespace muan

#endif  //  MUAN_PHOENIX_VICTOR_WRAPPER_H_
