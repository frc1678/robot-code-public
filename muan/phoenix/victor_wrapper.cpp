#include "muan/phoenix/victor_wrapper.h"
#include "muan/units/units.h"

namespace muan {
namespace phoenix {

VictorWrapper::VictorWrapper(int id, Config config)
    : victor_(id), conversion_factor_(config.conversion_factor) {
  LoadConfig(config);
}

void VictorWrapper::LoadConfig(Config config) {
  config_ = config;
  victor_.Set(ControlMode::PercentOutput, 0.);  // Safety first!

  switch (config.sensor) {
    case FeedbackSensor::kMagEncoderRelative:
      victor_.ConfigSelectedFeedbackSensor(
          FeedbackDevice::CTRE_MagEncoder_Relative, 0, kVictorSetupTimeout);
      victor_.ConfigSelectedFeedbackSensor(
          FeedbackDevice::CTRE_MagEncoder_Relative, 1, kVictorSetupTimeout);
      break;
    case FeedbackSensor::kMagEncoderAbsolute:
      victor_.ConfigSelectedFeedbackSensor(
          FeedbackDevice::CTRE_MagEncoder_Absolute, 0, kVictorSetupTimeout);
      victor_.ConfigSelectedFeedbackSensor(
          FeedbackDevice::CTRE_MagEncoder_Absolute, 1, kVictorSetupTimeout);
      break;
    case FeedbackSensor::kNone:
      break;
  }

  victor_.ChangeMotionControlFramePeriod(config.motion_control_frame_period);
  victor_.ClearMotionProfileHasUnderrun(kVictorSetupTimeout);
  victor_.ClearMotionProfileTrajectories();

  victor_.ClearStickyFaults(kVictorSetupTimeout);

  victor_.ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector,
                                        LimitSwitchNormal_NormallyOpen,
                                        kVictorSetupTimeout);
  victor_.ConfigReverseLimitSwitchSource(LimitSwitchSource_FeedbackConnector,
                                        LimitSwitchNormal_NormallyOpen,
                                        kVictorSetupTimeout);
  victor_.OverrideLimitSwitchesEnable(config.enable_limit_switch);

  victor_.ConfigSetParameter(ParamEnum::eClearPositionOnLimitF, 0, 0, 0,
                            kVictorSetupTimeout);
  victor_.ConfigSetParameter(ParamEnum::eClearPositionOnLimitR, 0, 0, 0,
                            kVictorSetupTimeout);

  victor_.ConfigNominalOutputForward(0, kVictorSetupTimeout);
  victor_.ConfigNominalOutputReverse(0, kVictorSetupTimeout);
  victor_.ConfigNeutralDeadband(config.neutral_deadband, kVictorSetupTimeout);

  victor_.ConfigPeakOutputForward(1., kVictorSetupTimeout);
  victor_.ConfigPeakOutputReverse(-1., kVictorSetupTimeout);

  victor_.SetNeutralMode(config.neutral_mode);

  victor_.ConfigForwardSoftLimitThreshold(
      config.forward_soft_limit * conversion_factor_, kVictorSetupTimeout);
  victor_.ConfigForwardSoftLimitEnable(config.enable_soft_limit,
                                      kVictorSetupTimeout);

  victor_.ConfigReverseSoftLimitThreshold(
      config.reverse_soft_limit * conversion_factor_, kVictorSetupTimeout);
  victor_.ConfigReverseSoftLimitEnable(config.enable_soft_limit,
                                      kVictorSetupTimeout);

  victor_.OverrideSoftLimitsEnable(config.enable_soft_limit);

  victor_.SetInverted(config.motor_inverted);
  victor_.SetSensorPhase(config.sensor_inverted);

  victor_.SelectProfileSlot(0, 0);

  victor_.ConfigVelocityMeasurementPeriod(config.velocity_measurement_period,
                                         kVictorSetupTimeout);
  victor_.ConfigVelocityMeasurementWindow(config.velocity_measurement_window,
                                         kVictorSetupTimeout);

  victor_.ConfigOpenloopRamp(config.open_loop_ramp_time, kVictorSetupTimeout);
  victor_.ConfigClosedloopRamp(config.closed_loop_ramp_time, kVictorSetupTimeout);

  victor_.ConfigVoltageCompSaturation(config.max_voltage, kVictorSetupTimeout);
  victor_.ConfigVoltageMeasurementFilter(32, kVictorSetupTimeout);
  victor_.EnableVoltageCompensation(config.compensate_voltage);

  victor_.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General,
                              config.general_status_frame_rate,
                              kVictorSetupTimeout);
  victor_.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0,
                              config.feedback_status_frame_rate,
                              kVictorSetupTimeout);
  victor_.SetStatusFramePeriod(StatusFrameEnhanced::Status_3_Quadrature,
                              config.quadrature_status_frame_rate,
                              kVictorSetupTimeout);
  victor_.SetStatusFramePeriod(StatusFrameEnhanced::Status_4_AinTempVbat,
                              config.analog_temp_vbat_status_frame_rate,
                              kVictorSetupTimeout);
  victor_.SetStatusFramePeriod(StatusFrameEnhanced::Status_8_PulseWidth,
                              config.pwm_status_frame_rate, kVictorSetupTimeout);

  victor_.SetControlFramePeriod(ControlFrame::Control_3_General,
                               config.control_frame_period);
}

void VictorWrapper::SetOpenloopGoal(double setpoint) {  // Voltage
  victor_.Set(ControlMode::PercentOutput, setpoint / config_.max_voltage);
}

void VictorWrapper::SetPositionGoal(double setpoint,
                                   double setpoint_ff) {  // Position, Voltage
  victor_.Set(ControlMode::Position, setpoint * conversion_factor_,
             DemandType_ArbitraryFeedForward, setpoint_ff * kVictorOutput);
}

void VictorWrapper::SetVelocityGoal(double setpoint,
                                   double setpoint_ff) {  // Velocity, Voltage
  victor_.Set(ControlMode::Velocity, setpoint * conversion_factor_ * 100 * ms,
             DemandType_ArbitraryFeedForward, setpoint_ff * kVictorOutput);
}

void VictorWrapper::SetGains(Gains gains, int slot) {
  victor_.Config_kP(slot, gains.p * kVictorOutput * conversion_factor_,
                   kVictorSetupTimeout);

  victor_.Config_kI(slot, gains.i * kVictorOutput * conversion_factor_ / ms,
                   kVictorSetupTimeout);

  victor_.Config_kD(slot, gains.d * kVictorOutput * conversion_factor_ * ms,
                   kVictorSetupTimeout);

  victor_.Config_kF(slot, gains.f * kVictorOutput * conversion_factor_,
                   kVictorSetupTimeout);

  victor_.ConfigMaxIntegralAccumulator(slot, gains.max_integral,
                                      kVictorSetupTimeout);
  victor_.Config_IntegralZone(slot, gains.i_zone, kVictorSetupTimeout);

  victor_.ConfigAllowableClosedloopError(slot, gains.deadband,
                                        kVictorSetupTimeout);
}

void VictorWrapper::SelectGains(int slot) { victor_.SelectProfileSlot(slot, 0); }

}  // namespace phoenix
}  // namespace muan
