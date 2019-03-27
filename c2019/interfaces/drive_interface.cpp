#include "c2019/interfaces/drive_interface.h"

namespace c2019 {
namespace interfaces {

using muan::queues::QueueManager;
using muan::subsystems::drivetrain::TalonOutput;

constexpr double kWheelRadius = 4.0 * 0.0254 / 2.0;
constexpr double kDriveConversionFactor = 4096 / (2. * M_PI * kWheelRadius);

constexpr uint32_t kShifter = 0;

constexpr int kVelocitySlot = 0;
constexpr int kPositionSlot = 1;
constexpr int kTurningSlot = 2;
constexpr int kSetupTimeout = 100;

constexpr double kHighGearPositionP = 0.6;
constexpr double kHighGearPositionI = 0;
constexpr double kHighGearPositionD = 6.;
constexpr double kHighGearPositionF = 0.;

constexpr double kHighGearVelocityP = 0.9;
constexpr double kHighGearVelocityI = 0;
constexpr double kHighGearVelocityD = 10.;
constexpr double kHighGearVelocityF = 0.12;

constexpr double kTurningP = 2.0;
constexpr double kTurningI = 0.0;
constexpr double kTurningD = 4.0;
constexpr double kTurningF = 0.0;

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

  left_master_.Config_kP(kTurningSlot, kTurningP, kSetupTimeout);
  left_master_.Config_kI(kTurningSlot, kTurningI, kSetupTimeout);
  left_master_.Config_kD(kTurningSlot, kTurningD, kSetupTimeout);
  left_master_.Config_kF(kTurningSlot, kTurningF, kSetupTimeout);
  left_master_.Config_IntegralZone(kTurningSlot, kIZone, kSetupTimeout);

  right_master_.Config_kP(kTurningSlot, kTurningP, kSetupTimeout);
  right_master_.Config_kI(kTurningSlot, kTurningI, kSetupTimeout);
  right_master_.Config_kD(kTurningSlot, kTurningD, kSetupTimeout);
  right_master_.Config_kF(kTurningSlot, kTurningF, kSetupTimeout);
  right_master_.Config_IntegralZone(kTurningSlot, 400, kSetupTimeout);

  /* right_master_.ConfigNeutralDeadband(0, kPositionSlot); */
  /* left_master_.ConfigNeutralDeadband(0, kPositionSlot); */

  /* right_master_.ConfigNeutralDeadband(0.04, kVelocitySlot); */
  /* left_master_.ConfigNeutralDeadband(0.04, kVelocitySlot); */
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
      pigeon_{&left_slave_a_},
      ds_status_reader_{QueueManager<muan::wpilib::DriverStationProto>::Fetch()
                            ->MakeReader()} {
  right_master_.ConfigFactoryDefault();
  left_master_.ConfigFactoryDefault();
  left_master_.ConfigSelectedFeedbackSensor(
      FeedbackDevice::CTRE_MagEncoder_Relative, kPositionSlot, kSetupTimeout);
  right_master_.ConfigSelectedFeedbackSensor(
      FeedbackDevice::CTRE_MagEncoder_Relative, kPositionSlot, kSetupTimeout);
  left_master_.ConfigSelectedFeedbackSensor(
      FeedbackDevice::CTRE_MagEncoder_Relative, kVelocitySlot, kSetupTimeout);
  right_master_.ConfigSelectedFeedbackSensor(
      FeedbackDevice::CTRE_MagEncoder_Relative, kVelocitySlot, kSetupTimeout);

  pigeon_.SetYaw(0, 100);

  /* right_master_.ConfigRemoteFeedbackFilter(left_master_.GetDeviceID(),
   * RemoteSensorSource::RemoteSensorSource_TalonSRX_SelectedSensor, 0, 100); */
  right_master_.ConfigRemoteFeedbackFilter(
      left_slave_a_.GetDeviceID(),
      RemoteSensorSource::RemoteSensorSource_GadgeteerPigeon_Yaw, 1, 100);

  /* right_master_.ConfigSensorTerm(SensorTerm::SensorTerm_Sum0,
   * FeedbackDevice::RemoteSensor0, 100); */
  /* right_master_.ConfigSensorTerm(SensorTerm::SensorTerm_Sum1,
   * FeedbackDevice::CTRE_MagEncoder_Relative, 100); */
  /* right_master_.ConfigSelectedFeedbackSensor(FeedbackDevice::SensorSum, 0,
   * 100); */
  right_master_.ConfigSelectedFeedbackSensor(FeedbackDevice::RemoteSensor1, 1,
                                             100);
  right_master_.ConfigSelectedFeedbackCoefficient((3600. / 8192.), 1, 100);

  left_master_.EnableVoltageCompensation(true);
  left_master_.ConfigVoltageCompSaturation(12.0, 100);
  left_master_.ConfigVoltageMeasurementFilter(32, 100);

  left_master_.ConfigMotionCruiseVelocity(2000, 100);
  right_master_.ConfigMotionCruiseVelocity(2000, 100);

  left_master_.ConfigMotionAcceleration(2000, 100);
  right_master_.ConfigMotionAcceleration(2000, 100);

  left_master_.SetSelectedSensorPosition(kPositionSlot, 0, kSetupTimeout);
  right_master_.SetSelectedSensorPosition(kPositionSlot, 0, kSetupTimeout);
  left_master_.SetSelectedSensorPosition(kVelocitySlot, 0, kSetupTimeout);
  right_master_.SetSelectedSensorPosition(kVelocitySlot, 0, kSetupTimeout);

  right_master_.ConfigAuxPIDPolarity(true, 100);
  right_master_.ConfigClosedLoopPeakOutput(kTurningSlot, 1.0, 100);

  left_master_.SetSensorPhase(true);

  left_master_.ConfigClosedloopRamp(kRampRate, kSetupTimeout);
  right_master_.ConfigClosedloopRamp(kRampRate, kSetupTimeout);

  left_slave_a_.Follow(left_master_);
  left_slave_b_.Follow(left_master_);

  right_slave_a_.Follow(right_master_);
  right_slave_b_.Follow(right_master_);

  right_master_.SetInverted(false);
  right_master_.SetSensorPhase(false);
  right_slave_a_.SetInverted(false);
  right_slave_b_.SetInverted(false);

  left_master_.SetInverted(true);
  left_master_.SetSensorPhase(false);
  left_slave_a_.SetInverted(true);
  left_slave_b_.SetInverted(true);

  LoadGains();
  SetBrakeMode(false);

  pigeon_offset_ = pigeon_.GetFusedHeading();

  right_master_.ConfigAllowableClosedloopError(kTurningSlot, 0, 100);
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

  sensors->set_left_current(left_master_.GetOutputCurrent());
  sensors->set_right_current(right_master_.GetOutputCurrent());

  sensors->set_left_voltage(left_master_.GetMotorOutputVoltage());
  sensors->set_right_voltage(right_master_.GetMotorOutputVoltage());

  sensors->set_right_bus(right_master_.GetBusVoltage());

  input_queue_->WriteMessage(sensors);
}

void DrivetrainInterface::WriteActuators() {
  OutputProto outputs;
  muan::wpilib::DriverStationProto ds;

  QueueManager<muan::wpilib::DriverStationProto>::Fetch()->ReadLastMessage(&ds);

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
      if (!compressor_.Enabled()) {
        compressor_.Start();
      }

      SetBrakeMode(false);
      left_master_.Set(ControlMode::PercentOutput, outputs->left_setpoint());
      right_master_.Set(ControlMode::PercentOutput, outputs->right_setpoint());
      break;
    case TalonOutput::POSITION:
      if (compressor_.Enabled()) {
        compressor_.Stop();
      }
      right_master_.SelectProfileSlot(kPositionSlot, 0);
      left_master_.SelectProfileSlot(kPositionSlot, 0);
      right_master_.Set(ControlMode::Position,
                        outputs->right_setpoint() * kDriveConversionFactor);
      left_master_.Set(ControlMode::Position,
                       outputs->left_setpoint() * kDriveConversionFactor);
      break;
    case TalonOutput::VELOCITY:
      if (compressor_.Enabled()) {
        compressor_.Stop();
      }
      SetBrakeMode(true);
      left_master_.SelectProfileSlot(kVelocitySlot, 0);
      right_master_.SelectProfileSlot(kVelocitySlot, 0);
      left_master_.Set(ControlMode::Velocity,
                       outputs->left_setpoint() * kDriveConversionFactor * 0.1);
      right_master_.Set(
          ControlMode::Velocity,
          outputs->right_setpoint() * kDriveConversionFactor * 0.1);
      break;
    case TalonOutput::ARC:
      if (compressor_.Enabled()) {
        compressor_.Stop();
      }
      right_master_.SelectProfileSlot(kPositionSlot, 0);
      right_master_.SelectProfileSlot(kTurningSlot, 1);
      right_master_.Set(ControlMode::PercentOutput, outputs->arc_vel() / 12.,
                        DemandType_AuxPID,
                        right_master_.GetSelectedSensorPosition(1) +
                            outputs->yaw() * (3600. / (2. * M_PI)));
      left_master_.Follow(right_master_, FollowerType::FollowerType_AuxOutput1);
      break;
  }

  // shifter_.Set(false);
}

}  // namespace interfaces
}  // namespace c2019
