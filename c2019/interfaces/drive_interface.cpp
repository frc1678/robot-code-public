#include "c2019/interfaces/drive_interface.h"

namespace c2019 {
namespace interfaces {

using muan::queues::QueueManager;
using muan::subsystems::drivetrain::TalonOutput;

constexpr double kWheelRadius = 4.0 * 0.0254 / 2.0;
constexpr double kDriveConversionFactor = 4096 / (2. * M_PI * kWheelRadius);

constexpr uint32_t kShifter = 0;

constexpr int kPositionSlot = 1;
constexpr int kVelocitySlot = 0;
constexpr int kSetupTimeout = 100;

constexpr double kHighGearPositionP = 1.2;
constexpr double kHighGearPositionI = 0;
constexpr double kHighGearPositionD = 6.;
constexpr double kHighGearPositionF = .15;

constexpr double kHighGearVelocityP = 0.9;
constexpr double kHighGearVelocityI = 0;
constexpr double kHighGearVelocityD = 10.;
constexpr double kHighGearVelocityF = 0;

constexpr double kIZone = 0;

constexpr double kRampRate = 0;

void DrivetrainInterface::LoadGains() {
  left_master_.Config_kP(kPositionSlot, kHighGearPositionP, kSetupTimeout);
  left_master_.Config_kI(kPositionSlot, kHighGearPositionI, kSetupTimeout);
  left_master_.Config_kD(kPositionSlot, kHighGearPositionD, kSetupTimeout);
  left_master_.Config_kF(kPositionSlot, kHighGearPositionF, kSetupTimeout);
  left_master_.Config_IntegralZone(kPositionSlot, kIZone, kSetupTimeout);

  right_master_.Config_kP(kPositionSlot, kHighGearPositionP, kSetupTimeout);
  right_master_.Config_kI(kPositionSlot, kHighGearPositionI, kSetupTimeout);
  right_master_.Config_kD(kPositionSlot, kHighGearPositionD, kSetupTimeout);
  right_master_.Config_kF(kPositionSlot, kHighGearPositionF, kSetupTimeout);
  right_master_.Config_IntegralZone(kPositionSlot, kIZone, kSetupTimeout);

  left_master_.Config_kP(kVelocitySlot, kHighGearVelocityP, kSetupTimeout);
  left_master_.Config_kI(kVelocitySlot, kHighGearVelocityI, kSetupTimeout);
  left_master_.Config_kD(kVelocitySlot, kHighGearVelocityD, kSetupTimeout);
  left_master_.Config_kF(kVelocitySlot, kHighGearVelocityF, kSetupTimeout);
  left_master_.Config_IntegralZone(kVelocitySlot, kIZone, kSetupTimeout);

  right_master_.Config_kP(kVelocitySlot, kHighGearVelocityP, kSetupTimeout);
  right_master_.Config_kI(kVelocitySlot, kHighGearVelocityI, kSetupTimeout);
  right_master_.Config_kD(kVelocitySlot, kHighGearVelocityD, kSetupTimeout);
  right_master_.Config_kF(kVelocitySlot, kHighGearVelocityF, kSetupTimeout);
  right_master_.Config_IntegralZone(kVelocitySlot, kIZone, kSetupTimeout);
}

void DrivetrainInterface::SetBrakeMode(bool mode) {
  NeutralMode neutral_mode = mode ? NeutralMode::Brake : NeutralMode::Coast;
  left_master_.SetNeutralMode(neutral_mode);
  left_slave_a_.SetNeutralMode(neutral_mode);
  left_slave_b_.SetNeutralMode(neutral_mode);

  right_master_.SetNeutralMode(neutral_mode);
  right_slave_a_.SetNeutralMode(neutral_mode);
  right_slave_b_.SetNeutralMode(neutral_mode);
}

DrivetrainInterface::DrivetrainInterface()
    : input_queue_{QueueManager<InputProto>::Fetch()},
      output_reader_{QueueManager<OutputProto>::Fetch()->MakeReader()},
      pigeon_{&right_slave_a_},
      ds_status_reader_{QueueManager<muan::wpilib::DriverStationProto>::Fetch()
                            ->MakeReader()} {
  left_master_.ConfigSelectedFeedbackSensor(
      FeedbackDevice::CTRE_MagEncoder_Relative, kPositionSlot, kSetupTimeout);
  right_master_.ConfigSelectedFeedbackSensor(
      FeedbackDevice::CTRE_MagEncoder_Relative, kPositionSlot, kSetupTimeout);
  left_master_.ConfigSelectedFeedbackSensor(
      FeedbackDevice::CTRE_MagEncoder_Relative, kVelocitySlot, kSetupTimeout);
  right_master_.ConfigSelectedFeedbackSensor(
      FeedbackDevice::CTRE_MagEncoder_Relative, kVelocitySlot, kSetupTimeout);

  left_master_.EnableVoltageCompensation(true);
  left_master_.ConfigVoltageCompSaturation(12.0, 100);
  left_master_.ConfigVoltageMeasurementFilter(32, 100);

  left_master_.SetSelectedSensorPosition(0, kPositionSlot, kSetupTimeout);
  right_master_.SetSelectedSensorPosition(0, kPositionSlot, kSetupTimeout);
  left_master_.SetSelectedSensorPosition(0, kVelocitySlot, kSetupTimeout);
  right_master_.SetSelectedSensorPosition(0, kVelocitySlot, kSetupTimeout);

  left_master_.SetSensorPhase(true);

  left_master_.ConfigClosedloopRamp(kRampRate, kSetupTimeout);
  right_master_.ConfigClosedloopRamp(kRampRate, kSetupTimeout);

  left_slave_a_.Follow(left_master_);
  left_slave_b_.Follow(left_master_);

  right_slave_a_.Follow(right_master_);
  right_slave_b_.Follow(right_master_);

  right_master_.SetInverted(true);
  right_master_.SetSensorPhase(true);
  right_slave_a_.SetInverted(true);
  right_slave_b_.SetInverted(true);

  LoadGains();
  SetBrakeMode(false);

  pigeon_offset_ = pigeon_.GetFusedHeading();
}

void DrivetrainInterface::ReadSensors() {
  InputProto sensors;

  sensors->set_left_encoder(left_master_.GetSelectedSensorPosition(0) /
                            kDriveConversionFactor);
  sensors->set_right_encoder(right_master_.GetSelectedSensorPosition(0) /
                             kDriveConversionFactor);
  sensors->set_left_velocity(left_master_.GetSelectedSensorVelocity(0) /
                             kDriveConversionFactor / 0.1);
  sensors->set_right_velocity(right_master_.GetSelectedSensorVelocity(0) /
                              kDriveConversionFactor / 0.1);

  sensors->set_gyro(-(pigeon_.GetFusedHeading() - pigeon_offset_) * M_PI /
                    180.);

  input_queue_->WriteMessage(sensors);
}

void DrivetrainInterface::WriteActuators() {
  OutputProto outputs;
  muan::wpilib::DriverStationProto ds;

  if (!output_reader_.ReadLastMessage(&outputs)) {
    left_master_.Set(ControlMode::PercentOutput, 0);
    right_master_.Set(ControlMode::PercentOutput, 0);
    SetBrakeMode(false);
    return;
  }

  if (!ds->is_sys_active()) {
    SetBrakeMode(false);
  }

  switch (outputs->output_type()) {
    case TalonOutput::OPEN_LOOP:
      SetBrakeMode(false);
      left_master_.Set(ControlMode::PercentOutput, outputs->left_setpoint());
      right_master_.Set(ControlMode::PercentOutput, outputs->right_setpoint());
      break;
    case TalonOutput::POSITION:
      left_master_.SelectProfileSlot(kPositionSlot, 0);
      right_master_.SelectProfileSlot(kPositionSlot, 0);
      left_master_.Set(ControlMode::Position, outputs->left_setpoint() * kDriveConversionFactor);
      right_master_.Set(ControlMode::Position, outputs->right_setpoint() * kDriveConversionFactor);
      break;
    case TalonOutput::VELOCITY:
      SetBrakeMode(true);
      left_master_.SelectProfileSlot(kVelocitySlot, 0);
      right_master_.SelectProfileSlot(kVelocitySlot, 0);
      left_master_.Set(ControlMode::Velocity,
                       outputs->left_setpoint() * kDriveConversionFactor * 0.1);
                       /* DemandType_ArbitraryFeedForward, */
                       /* outputs->left_setpoint_ff() / 12.); */
      right_master_.Set(
          ControlMode::Velocity,
          outputs->right_setpoint() * kDriveConversionFactor * 0.1);
          /* DemandType_ArbitraryFeedForward, outputs->right_setpoint_ff() / 12.); */
      break;
  }

  shifter_.Set(outputs->high_gear());
}

}  // namespace interfaces
}  // namespace c2019
