#include "o2018/interfaces/arm_interface.h"

namespace o2018 {
namespace interfaces {

using o2018::subsystems::arm::TalonOutput;

constexpr double kSensorRatio = 1.0;
constexpr double kArmConversionFactor = (4096 * 6.42857) / (2 * M_PI);

constexpr double kP = 3.;
constexpr double kI = 0.;
constexpr double kD = 50.;
constexpr double kF = 1.05;
constexpr double kIZone = 0.;
constexpr double kMaxIntegral = 5e9;
constexpr double kDeadband = 0.001;

void ArmInterface::LoadGains() {
  arm_talon_.Config_kP(0, kP, 100);
  arm_talon_.Config_kI(0, kI, 100);
  arm_talon_.Config_kD(0, kD, 100);
  arm_talon_.Config_kF(0, kF, 100);
  arm_talon_.Config_IntegralZone(0, kIZone, 100);
}

ArmInterface::ArmInterface() {
  LoadGains();

  arm_talon_.ConfigSelectedFeedbackSensor(
      FeedbackDevice::CTRE_MagEncoder_Relative, 0, 100);
  arm_talon_.SetSelectedSensorPosition(0, 0, 100);

  arm_talon_.SetSensorPhase(true);

  intake_right_.SetInverted(true);
  intake_right_.ConfigReverseLimitSwitchSource(
      LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyClosed,
      100);
  intake_right_.ConfigForwardLimitSwitchSource(
      LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen, 100);
  arm_talon_.ConfigMotionCruiseVelocity(1000, 100);
  arm_talon_.ConfigMotionAcceleration(2000, 100);
}

void ArmInterface::ReadSensors() {
  ArmInputProto sensors;

  sensors->set_arm_encoder(arm_talon_.GetSelectedSensorPosition(0) /
                           kArmConversionFactor);
  sensors->set_arm_velocity(arm_talon_.GetSelectedSensorVelocity(0) /
                            kArmConversionFactor / 0.1);
  sensors->set_arm_hall(!arm_hall_.Get());

  if (sensors->arm_hall() && !has_been_zeroed_) {
    arm_talon_.SetSelectedSensorPosition(0, 0, 100);
    has_been_zeroed_ = true;
  }

  sensors->set_zeroed(has_been_zeroed_);

  sensors->set_intake_proxy(intake_proxy_.Get());

  input_queue_->WriteMessage(sensors);
}

void ArmInterface::WriteActuators() {
  ArmOutputProto outputs;
  if (!output_reader_.ReadLastMessage(&outputs)) {
    arm_talon_.Set(ControlMode::PercentOutput, 0);
    intake_left_.Set(ControlMode::PercentOutput, 0);
    intake_right_.Set(ControlMode::PercentOutput, 0);

    intake_open_.Set(false);
    intake_close_.Set(false);
    return;
  }

  intake_right_.Set(ControlMode::PercentOutput,
                    outputs->intake_voltage() / 12.);
  intake_left_.Set(ControlMode::PercentOutput, outputs->intake_voltage() / 12.);
  intake_open_.Set(outputs->intake_open());
  intake_close_.Set(!outputs->intake_close());

  switch (outputs->arm_output_type()) {
    case TalonOutput::OPEN_LOOP:
      arm_talon_.Set(ControlMode::PercentOutput, outputs->arm_setpoint() / 12.);
      break;
    case TalonOutput::POSITION:
      arm_talon_.Set(ControlMode::MotionMagic,
                     outputs->arm_setpoint() * kArmConversionFactor,
                     DemandType_ArbitraryFeedForward,
                     outputs->arm_setpoint_ff() / 12.);
      break;
  }
}

}  // namespace interfaces
}  // namespace o2018
