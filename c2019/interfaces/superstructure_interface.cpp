#include "c2019/interfaces/superstructure_interface.h"

namespace c2019 {
namespace interfaces {

constexpr double kElevatorConversionFactor =
    (4096) / (M_PI * 1.25 * 0.0254 * 1.6);

constexpr double kWristConversionFactor = (4096 * 2.933) / (2 * M_PI);

constexpr double kElevatorP = 0.12;
constexpr double kElevatorI = 0.0;
constexpr double kElevatorD = 4.0;
constexpr double kElevatorF = 0.06;
constexpr double kElevatorIZone = 0.;
constexpr double kElevatorMaxIntegral = 5e9;
constexpr double kElevatorDeadband = 0.0;

constexpr double kWristP = 1.0;
constexpr double kWristI = 0.0;
constexpr double kWristD = 20.0;
constexpr double kWristF = 0.3;
constexpr double kWristIZone = 0.;
constexpr double kWristMaxIntegral = 5e9;
constexpr double kWristDeadband = 0.0;

constexpr uint32_t kGroundPDPSlot = 7;
constexpr uint32_t kCargoPDPSlot = 6;

constexpr uint32_t kSetupTimeout = 100;

constexpr uint32_t kElevatorCruiseVelocity = 14325;
constexpr uint32_t kElevatorCruiseAccel = 19100;
constexpr uint32_t KWristCruiseVelocity = 1432.5;
constexpr uint32_t KWristCruiseAccel = 1528;


using c2019::superstructure::TalonOutput;
using muan::queues::QueueManager;

SuperstructureInterface::SuperstructureInterface(
    muan::wpilib::CanWrapper* can_wrapper)
    : input_queue_{QueueManager<SuperstructureInputProto>::Fetch()},
      output_reader_{
          QueueManager<SuperstructureOutputProto>::Fetch()->MakeReader()},
      pcm_{can_wrapper->pcm()} {
  elevator_master_.ConfigFactoryDefault();
  elevator_slave_a_.ConfigFactoryDefault();
  elevator_slave_b_.ConfigFactoryDefault();
  elevator_slave_c_.ConfigFactoryDefault();

  wrist_.ConfigFactoryDefault();
  canifier_.ConfigFactoryDefault();

  LoadGains();
  pcm_->CreateSolenoid(kArrow);
  pcm_->CreateSolenoid(kBackplate);
  pcm_->CreateSolenoid(kCrawlerOne);
  pcm_->CreateSolenoid(kForkDrop);
  pcm_->CreateSolenoid(kShifter);
  pcm_->CreateSolenoid(kCargo);
  pcm_->CreateSolenoid(kPins);

  elevator_master_.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General,
                                        20, kSetupTimeout);
  elevator_master_.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0,
                                        20, kSetupTimeout);

  elevator_slave_a_.SetStatusFramePeriod(StatusFrame::Status_1_General_, kSetupTimeout,
                                         kSetupTimeout);
  elevator_slave_a_.SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, kSetupTimeout,
                                         kSetupTimeout);

  elevator_slave_b_.SetStatusFramePeriod(StatusFrame::Status_1_General_, kSetupTimeout,
                                         kSetupTimeout);
  elevator_slave_b_.SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, kSetupTimeout,
                                         kSetupTimeout);

  elevator_slave_c_.SetStatusFramePeriod(StatusFrame::Status_1_General_, kSetupTimeout,
                                         kSetupTimeout);
  elevator_slave_c_.SetStatusFramePeriod(StatusFrame::Status_2_Feedback0_, kSetupTimeout,
                                         kSetupTimeout);

  wrist_.SetStatusFramePeriod(StatusFrameEnhanced::Status_1_General, 20, kSetupTimeout);
  wrist_.SetStatusFramePeriod(StatusFrameEnhanced::Status_2_Feedback0, 20, kSetupTimeout);
}

void SuperstructureInterface::ReadSensors() {
  SuperstructureInputProto inputs;
  muan::wpilib::PdpMessage pdp;
  QueueManager<muan::wpilib::PdpMessage>::Fetch()->ReadLastMessage(&pdp);

  inputs->set_wrist_current(wrist_.GetOutputCurrent());
  inputs->set_elevator_current(pdp->current3());
  inputs->set_wrist_voltage(wrist_.GetMotorOutputVoltage());
  inputs->set_elevator_voltage(elevator_master_.GetMotorOutputVoltage());
  inputs->set_elevator_encoder(elevator_master_.GetSelectedSensorPosition() /
                               kElevatorConversionFactor);

  inputs->set_elevator_hall(
      elevator_master_.GetSensorCollection().IsRevLimitSwitchClosed());

  if (inputs->elevator_hall() && !elevator_zeroed_) {
    elevator_zeroed_ = true;
    elevator_master_.SetSelectedSensorPosition(0, 0, kSetupTimeout);
  }

  inputs->set_wrist_encoder(wrist_.GetSelectedSensorPosition() /
                            kWristConversionFactor);

  inputs->set_elevator_zeroed(elevator_zeroed_);

  inputs->set_wrist_hall(
      !canifier_.GetGeneralInput(CANifier::GeneralPin::LIMR));

  if (inputs->wrist_hall() && !wrist_zeroed_) {
    wrist_zeroed_ = true;
    wrist_.SetSelectedSensorPosition(0, 0, kSetupTimeout);
    canifier_.SetQuadraturePosition(0, 0);
  }

  inputs->set_wrist_zeroed(wrist_zeroed_);

  inputs->set_cargo_proxy(
      canifier_.GetGeneralInput(CANifier::GeneralPin::SPI_CLK_PWM0P));
  inputs->set_hatch_intake_proxy(
      canifier_.GetGeneralInput(CANifier::GeneralPin::SPI_MOSI_PWM1P) &&
      canifier_.GetGeneralInput(CANifier::GeneralPin::SPI_CS));

  if (elevator_master_.GetControlMode() == ControlMode::MotionMagic) {
    inputs->set_elevator_profiled_goal(
        elevator_master_.GetActiveTrajectoryPosition());
    inputs->set_elevator_profiled_velocity(
        elevator_master_.GetActiveTrajectoryVelocity());
  }

  if (wrist_.GetControlMode() == ControlMode::MotionMagic) {
    inputs->set_wrist_profiled_goal(wrist_.GetActiveTrajectoryPosition());
    inputs->set_wrist_profiled_velocity(wrist_.GetActiveTrajectoryVelocity());
  }

  input_queue_->WriteMessage(inputs);
}

void SuperstructureInterface::LoadGains() {
  elevator_master_.Config_kP(0, kElevatorP, kSetupTimeout);
  elevator_master_.Config_kI(0, kElevatorI, kSetupTimeout);
  elevator_master_.Config_kD(0, kElevatorD, kSetupTimeout);
  elevator_master_.Config_kF(0, kElevatorF, kSetupTimeout);
  elevator_master_.Config_IntegralZone(0, kElevatorIZone, kSetupTimeout);

  wrist_.Config_kP(0, kWristP, kSetupTimeout);
  wrist_.Config_kI(0, kWristI, kSetupTimeout);
  wrist_.Config_kD(0, kWristD, kSetupTimeout);
  wrist_.Config_kF(0, kWristF, kSetupTimeout);
  wrist_.Config_IntegralZone(0, kWristIZone, kSetupTimeout);

  wrist_.ConfigReverseLimitSwitchSource(RemoteLimitSwitchSource_RemoteCANifier,
                                        LimitSwitchNormal_NormallyOpen,
                                        kCANifier, kSetupTimeout);
  wrist_.ConfigForwardLimitSwitchSource(LimitSwitchSource_FeedbackConnector,
                                        LimitSwitchNormal_NormallyOpen, kSetupTimeout);
  elevator_master_.ConfigSelectedFeedbackSensor(
      FeedbackDevice::CTRE_MagEncoder_Relative, 0, kSetupTimeout);

  wrist_.ConfigRemoteFeedbackFilter(
      0, RemoteSensorSource::RemoteSensorSource_CANifier_Quadrature, kCANifier,
      kSetupTimeout);

  wrist_.ConfigSelectedFeedbackSensor(RemoteFeedbackDevice_RemoteSensor0, 0,
                                      kSetupTimeout);

  elevator_master_.SetSensorPhase(false);

  constexpr bool elevator_inverted = false;
  elevator_master_.SetInverted(elevator_inverted);

  elevator_master_.ConfigReverseLimitSwitchSource(
      LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen, kSetupTimeout);
  elevator_master_.ConfigForwardLimitSwitchSource(
      LimitSwitchSource_FeedbackConnector, LimitSwitchNormal_NormallyOpen, kSetupTimeout);
  elevator_master_.ConfigMotionCruiseVelocity(kElevatorCruiseVelocity, kSetupTimeout);
  elevator_master_.ConfigMotionAcceleration(kElevatorCruiseAccel, kSetupTimeout);
  wrist_.ConfigMotionCruiseVelocity(KWristCruiseVelocity, kSetupTimeout);
  wrist_.ConfigMotionAcceleration(KWristCruiseAccel, kSetupTimeout);
  elevator_master_.ConfigAllowableClosedloopError(0, kElevatorDeadband, 0);

  elevator_master_.ConfigForwardSoftLimitEnable(false);
  elevator_master_.ConfigReverseSoftLimitEnable(false);

  elevator_slave_a_.Follow(elevator_master_);
  elevator_slave_a_.SetInverted(elevator_inverted);
  elevator_slave_b_.Follow(elevator_master_);
  elevator_slave_b_.SetInverted(elevator_inverted);
  elevator_slave_c_.Follow(elevator_master_);
  elevator_slave_c_.SetInverted(elevator_inverted);

  elevator_master_.OverrideLimitSwitchesEnable(true);
  elevator_master_.OverrideSoftLimitsEnable(true);

  wrist_.OverrideLimitSwitchesEnable(true);
  wrist_.OverrideSoftLimitsEnable(true);

  winch_two_.SetInverted(true);
}

void SuperstructureInterface::SetBrakeMode(bool mode) {
  NeutralMode neutral_mode = mode ? NeutralMode::Brake : NeutralMode::Coast;
  winch_two_.SetNeutralMode(neutral_mode);
  elevator_master_.SetNeutralMode(neutral_mode);
  elevator_slave_a_.SetNeutralMode(neutral_mode);
  elevator_slave_b_.SetNeutralMode(neutral_mode);
  elevator_slave_c_.SetNeutralMode(neutral_mode);
  wrist_.SetNeutralMode(neutral_mode);
  winch_.SetNeutralMode(neutral_mode);
}

void SuperstructureInterface::WriteActuators() {
  SuperstructureOutputProto outputs;
  muan::wpilib::DriverStationProto ds;

  QueueManager<muan::wpilib::DriverStationProto>::Fetch()->ReadLastMessage(&ds);

  if (!output_reader_.ReadLastMessage(&outputs)) {
    elevator_master_.Set(ControlMode::PercentOutput, 0);
    wrist_.Set(ControlMode::PercentOutput, 0);

    cargo_intake_.Set(ControlMode::PercentOutput, 0);
    crawler_.Set(ControlMode::PercentOutput, 0);
    winch_.Set(ControlMode::PercentOutput, 0);
    winch_two_.Set(ControlMode::PercentOutput, 0);

    pcm_->WriteSolenoid(kArrow, false);
    pcm_->WriteSolenoid(kBackplate, false);
    pcm_->WriteSolenoid(kCrawlerOne, false);
    pcm_->WriteSolenoid(kForkDrop, false);
    pcm_->WriteSolenoid(kShifter, false);
    pcm_->WriteSolenoid(kCargo, false);
    pcm_->WriteSolenoid(kPins, false);

    SetBrakeMode(false);
    return;
  }

  SetBrakeMode(ds->is_sys_active());

  switch (outputs->elevator_setpoint_type()) {
    case TalonOutput::OPEN_LOOP:
      elevator_master_.Set(ControlMode::PercentOutput,
                           outputs->elevator_setpoint() / 12.);
      break;
    case TalonOutput::POSITION:
      elevator_master_.Set(
          ControlMode::MotionMagic,
          outputs->elevator_setpoint() * kElevatorConversionFactor,
          DemandType_ArbitraryFeedForward,
          outputs->elevator_setpoint_ff() / 12.);
      break;
  }

  switch (outputs->wrist_setpoint_type()) {
    case TalonOutput::OPEN_LOOP:
      wrist_.Set(ControlMode::PercentOutput, outputs->wrist_setpoint() / 12.);
      break;
    case TalonOutput::POSITION:
      wrist_.Set(ControlMode::MotionMagic,
                 outputs->wrist_setpoint() * kWristConversionFactor,
                 DemandType_ArbitraryFeedForward,
                 outputs->wrist_setpoint_ff() / 12.);
      break;
  }

  cargo_intake_.Set(ControlMode::PercentOutput,
                    -outputs->cargo_roller_voltage() / 12.);
  crawler_.Set(ControlMode::PercentOutput, -outputs->crawler_voltage() / 12.);
  winch_.Set(ControlMode::PercentOutput, outputs->left_winch_voltage() / 12.);
  winch_two_.Set(ControlMode::PercentOutput,
                 outputs->right_winch_voltage() / 12.);

  pcm_->WriteSolenoid(kArrow, !outputs->arrow_solenoid());
  pcm_->WriteSolenoid(kBackplate, outputs->backplate_solenoid());
  pcm_->WriteSolenoid(kCrawlerOne, outputs->crawler_one_solenoid());
  pcm_->WriteSolenoid(kForkDrop, outputs->drop_forks());
  pcm_->WriteSolenoid(kShifter, !outputs->elevator_high_gear());
  pcm_->WriteSolenoid(kCargo, outputs->cargo_out());
  pcm_->WriteSolenoid(kPins, outputs->pins());
}

}  // namespace interfaces
}  // namespace c2019
