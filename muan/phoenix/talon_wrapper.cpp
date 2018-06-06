#include "muan/phoenix/talon_wrapper.h"
#include "muan/units/units.h"

namespace muan {
namespace phoenix {

TalonWrapper::TalonWrapper(int id, Config config)
    : talon_(id), conversion_factor_(config.conversion_factor) {
  LoadConfig(config);
}

void TalonWrapper::LoadConfig(Config config) {
  config_ = config;
  talon_.Set(ControlMode::PercentOutput, 0.);  // Safety first!

  switch (config.sensor) {
    case FeedbackSensor::kMagEncoderRelative:
      talon_.ConfigSelectedFeedbackSensor(
          FeedbackDevice::CTRE_MagEncoder_Relative, 0, kTalonSetupTimeout);
      talon_.ConfigSelectedFeedbackSensor(
          FeedbackDevice::CTRE_MagEncoder_Relative, 1, kTalonSetupTimeout);
      break;
    case FeedbackSensor::kMagEncoderAbsolute:
      talon_.ConfigSelectedFeedbackSensor(
          FeedbackDevice::CTRE_MagEncoder_Absolute, 0, kTalonSetupTimeout);
      talon_.ConfigSelectedFeedbackSensor(
          FeedbackDevice::CTRE_MagEncoder_Absolute, 1, kTalonSetupTimeout);
      break;
    case FeedbackSensor::kNone:
      break;
  }

  talon_.ChangeMotionControlFramePeriod(config.motion_control_frame_period);
  talon_.ClearMotionProfileHasUnderrun(kTalonSetupTimeout);
  talon_.ClearMotionProfileTrajectories();

  talon_.ClearStickyFaults(kTalonSetupTimeout);

  talon_.ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector,
                                        LimitSwitchNormal_NormallyOpen,
                                        kTalonSetupTimeout);
  talon_.ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector,
                                        LimitSwitchNormal_NormallyOpen,
                                        kTalonSetupTimeout);
  talon_.OverrideLimitSwitchesEnable(config.enable_limit_switch);

  talon_.ConfigSetParameter(ParamEnum::eClearPositionOnLimitF, 0, 0, 0,
                            kTalonSetupTimeout);
  talon_.ConfigSetParameter(ParamEnum::eClearPositionOnLimitR, 0, 0, 0,
                            kTalonSetupTimeout);

  talon_.ConfigNominalOutputForward(0, kTalonSetupTimeout);
  talon_.ConfigNominalOutputReverse(0, kTalonSetupTimeout);
  talon_.ConfigNeutralDeadband(config.neutral_deadband, kTalonSetupTimeout);

  talon_.ConfigPeakOutputForward(1., kTalonSetupTimeout);
  talon_.ConfigPeakOutputReverse(-1., kTalonSetupTimeout);

  talon_.SetNeutralMode(config.neutral_mode);

  talon_.ConfigForwardSoftLimitThreshold(
      config.forward_soft_limit * conversion_factor_, kTalonSetupTimeout);
  talon_.ConfigForwardSoftLimitEnable(config.enable_soft_limit,
                                      kTalonSetupTimeout);

  talon_.ConfigReverseSoftLimitThreshold(
      config.reverse_soft_limit * conversion_factor_, kTalonSetupTimeout);
  talon_.ConfigReverseSoftLimitEnable(config.enable_soft_limit,
                                      kTalonSetupTimeout);

  talon_.OverrideSoftLimitsEnable(config.enable_soft_limit);

  talon_.SetInverted(config.motor_inverted);
  talon_.SetSensorPhase(config.sensor_inverted);

  talon_.SelectProfileSlot(0, 0);

  talon_.ConfigVelocityMeasurementPeriod(config.velocity_measurement_period,
                                         kTalonSetupTimeout);
  talon_.ConfigVelocityMeasurementWindow(config.velocity_measurement_window,
                                         kTalonSetupTimeout);

  talon_.ConfigOpenloopRamp(config.open_loop_ramp_time, kTalonSetupTimeout);
  talon_.ConfigClosedloopRamp(config.closed_loop_ramp_time, kTalonSetupTimeout);

  talon_.ConfigVoltageCompSaturation(config.max_voltage, kTalonSetupTimeout);
  talon_.ConfigVoltageMeasurementFilter(32, kTalonSetupTimeout);
  talon_.EnableVoltageCompensation(config.compensate_voltage);

  talon_.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General,
                              config.general_status_frame_rate,
                              kTalonSetupTimeout);
  talon_.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0,
                              config.feedback_status_frame_rate,
                              kTalonSetupTimeout);
  talon_.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature,
                              config.quadrature_status_frame_rate,
                              kTalonSetupTimeout);
  talon_.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat,
                              config.analog_temp_vbat_status_frame_rate,
                              kTalonSetupTimeout);
  talon_.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth,
                              config.pwm_status_frame_rate, kTalonSetupTimeout);

  talon_.SetControlFramePeriod(ControlFrame::Control_3_General,
                               config.control_frame_period);
}

void TalonWrapper::SetOpenloopGoal(double setpoint) {  // Voltage
  talon_.Set(ControlMode::PercentOutput, setpoint / config_.max_voltage);
}

void TalonWrapper::SetPositionGoal(double setpoint,
                                   double setpoint_ff) {  // Position, Voltage
  talon_.Set(ControlMode::Position, setpoint * conversion_factor_,
             DemandType_ArbitraryFeedForward, setpoint_ff * kTalonOutput);
}

void TalonWrapper::SetVelocityGoal(double setpoint,
                                   double setpoint_ff) {  // Velocity, Voltage
  talon_.Set(ControlMode::Velocity, setpoint * conversion_factor_ * 100 * ms,
             DemandType_ArbitraryFeedForward, setpoint_ff * kTalonOutput);
}

void TalonWrapper::SetGains(Gains gains, int slot) {
  talon_.Config_kP(slot, gains.p * kTalonOutput * conversion_factor_,
                   kTalonSetupTimeout);

  talon_.Config_kI(slot, gains.i * kTalonOutput * conversion_factor_ / ms,
                   kTalonSetupTimeout);

  talon_.Config_kD(slot, gains.d * kTalonOutput * conversion_factor_ * ms,
                   kTalonSetupTimeout);

  talon_.Config_kF(slot, gains.f * kTalonOutput * conversion_factor_,
                   kTalonSetupTimeout);

  talon_.ConfigMaxIntegralAccumulator(slot, gains.max_integral,
                                      kTalonSetupTimeout);
  talon_.Config_IntegralZone(slot, gains.i_zone, kTalonSetupTimeout);

  talon_.ConfigAllowableClosedloopError(slot, gains.deadband,
                                        kTalonSetupTimeout);
}

void TalonWrapper::SelectGains(int slot) { talon_.SelectProfileSlot(slot, 0); }

}  // namespace phoenix
}  // namespace muan
