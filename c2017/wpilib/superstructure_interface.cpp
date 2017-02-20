#include "c2017/wpilib/superstructure_interface.h"

namespace c2017 {

namespace wpilib {

namespace ports {

namespace superstructure {

// TODO(Kelly) figure out the correct ports for everything
// Motor ports
constexpr uint32_t kShooterMotor = 2;

constexpr uint32_t kUpperConveyorMotor = 5;
constexpr uint32_t kSideConveyorMotor = 4;

constexpr uint32_t kBallIntakeMotor = 3;

constexpr uint32_t kGearIntakeMotor = 6;

// Sensor ports
constexpr uint32_t kShooterEncoderA = 20, kShooterEncoderB = 21;

// Solenoid ports
constexpr uint32_t kBallIntakeSolenoid = 7;
constexpr uint32_t kGroundGearIntakeSolenoidA = 2;
constexpr uint32_t kGroundGearIntakeSolenoidB = 3;
constexpr uint32_t kGearShutterSolenoid = 5;
constexpr uint32_t kHpGearIntakeSolenoid = 4;
constexpr uint32_t kMagazineSolenoidA = 0;
constexpr uint32_t kMagazineSolenoidB = 1;
constexpr uint32_t kShooterHoodSolenoid = 6;

// Other
constexpr double kMaxVoltage = 12;

}  // namespace superstructure

}  // namespace ports

SuperStructureInterface::SuperStructureInterface(muan::wpilib::CanWrapper* can_wrapper)
    : output_queue_(QueueManager::GetInstance().superstructure_output_queue().MakeReader()),
      shooter_motor_{ports::superstructure::kShooterMotor},
      upper_conveyor_motor_{ports::superstructure::kUpperConveyorMotor},
      side_conveyor_motor_{ports::superstructure::kSideConveyorMotor},
      ball_intake_motor_{ports::superstructure::kBallIntakeMotor},
      gear_intake_motor_{ports::superstructure::kGearIntakeMotor},
      shooter_encoder_{ports::superstructure::kShooterEncoderA, ports::superstructure::kShooterEncoderB},
      pcm_{can_wrapper->pcm()} {
  pcm_->CreateSolenoid(ports::superstructure::kBallIntakeSolenoid);
  pcm_->CreateDoubleSolenoid(ports::superstructure::kGroundGearIntakeSolenoidA,
                             ports::superstructure::kGroundGearIntakeSolenoidB);
  pcm_->CreateSolenoid(ports::superstructure::kGearShutterSolenoid);
  pcm_->CreateSolenoid(ports::superstructure::kHpGearIntakeSolenoid);
  pcm_->CreateDoubleSolenoid(ports::superstructure::kMagazineSolenoidA,
                             ports::superstructure::kMagazineSolenoidB);
  pcm_->CreateSolenoid(ports::superstructure::kShooterHoodSolenoid);
}

void SuperStructureInterface::ReadSensors() {
  c2017::shooter::ShooterInputProto shooter_sensors;
  c2017::climber::ClimberInputProto climber_sensors;

  c2017::magazine::MagazineInputProto magazine_sensors;
  c2017::ground_gear_intake::GroundGearIntakeInputProto ground_gear_sensors;

  constexpr double kRadiansPerClick = M_PI * 2 / 512.0;

  shooter_sensors->set_encoder_position(shooter_encoder_.Get() * kRadiansPerClick);
  climber_sensors->set_position(shooter_encoder_.Get() * kRadiansPerClick);

  auto current_reader = QueueManager::GetInstance().pdp_status_queue().MakeReader().ReadLastMessage();

  if (current_reader) {
    // Place holder for now
    // TODO(Kelly) figure out what exactly needs current inputs and what ports it is in
    // Ground gear intake, climber
    climber_sensors->set_current((*current_reader)->current5());
    ground_gear_sensors->set_current((*current_reader)->current11());
  }


  QueueManager::GetInstance().shooter_input_queue().WriteMessage(shooter_sensors);
  QueueManager::GetInstance().climber_input_queue().WriteMessage(climber_sensors);
  QueueManager::GetInstance().magazine_input_queue().WriteMessage(magazine_sensors);
  QueueManager::GetInstance().ground_gear_input_queue().WriteMessage(ground_gear_sensors);
}

void SuperStructureInterface::WriteActuators() {
  auto outputs = output_queue_.ReadLastMessage();
  auto current_reader = QueueManager::GetInstance().pdp_status_queue().MakeReader().ReadLastMessage();
  if (outputs) {
    // Shooter motors
    shooter_motor_.Set(-muan::utils::Cap(-(*outputs)->shooter_voltage(), -ports::superstructure::kMaxVoltage,
                                         ports::superstructure::kMaxVoltage) /
                       12.0);

    // Upper Conveyor motor
    upper_conveyor_motor_.Set(-muan::utils::Cap((*outputs)->upper_conveyor_voltage(),
                                                -ports::superstructure::kMaxVoltage,
                                                ports::superstructure::kMaxVoltage) /
                              12.0);
    // Side Conveyor motor
    side_conveyor_motor_.Set(-muan::utils::Cap((*outputs)->side_conveyor_voltage(),
                                               -ports::superstructure::kMaxVoltage,
                                               ports::superstructure::kMaxVoltage) /
                             12.0);

    // Main ball intake motors
    ball_intake_motor_.Set(-muan::utils::Cap((*outputs)->main_roller_voltage(),
                                             -ports::superstructure::kMaxVoltage,
                                             ports::superstructure::kMaxVoltage) /
                           12.0);

    // Ground gear intake motor
    gear_intake_motor_.Set(-muan::utils::Cap((*outputs)->ground_gear_voltage(),
                                             -ports::superstructure::kMaxVoltage,
                                             ports::superstructure::kMaxVoltage) /
                           12.0);

    // Solenoids
    pcm_->WriteSolenoid(ports::superstructure::kBallIntakeSolenoid, (*outputs)->ball_intake_down());
    pcm_->WriteDoubleSolenoid(
        ports::superstructure::kGroundGearIntakeSolenoidA, ports::superstructure::kGroundGearIntakeSolenoidB,
        (*outputs)->ground_gear_down() ? DoubleSolenoid::Value::kReverse : DoubleSolenoid::Value::kForward);
    pcm_->WriteSolenoid(ports::superstructure::kGearShutterSolenoid, (*outputs)->gear_shutter_open());
    pcm_->WriteSolenoid(ports::superstructure::kHpGearIntakeSolenoid, (*outputs)->hp_gear_open());
    pcm_->WriteDoubleSolenoid(
        ports::superstructure::kMagazineSolenoidA, ports::superstructure::kMagazineSolenoidB,
        (*outputs)->magazine_open() ? DoubleSolenoid::Value::kReverse : DoubleSolenoid::Value::kForward);
    pcm_->WriteSolenoid(ports::superstructure::kShooterHoodSolenoid, (*outputs)->shooter_hood_up());

  } else {
    shooter_motor_.Set(0);
    upper_conveyor_motor_.Set(0);
    side_conveyor_motor_.Set(0);
    ball_intake_motor_.Set(0);
    gear_intake_motor_.Set(0);

    pcm_->WriteSolenoid(ports::superstructure::kBallIntakeSolenoid, false);
    pcm_->WriteDoubleSolenoid(ports::superstructure::kGroundGearIntakeSolenoidA,
                              ports::superstructure::kGroundGearIntakeSolenoidB,
                              DoubleSolenoid::Value::kForward);
    pcm_->WriteSolenoid(ports::superstructure::kGearShutterSolenoid, false);
    pcm_->WriteSolenoid(ports::superstructure::kHpGearIntakeSolenoid, false);
    pcm_->WriteDoubleSolenoid(ports::superstructure::kMagazineSolenoidA,
                              ports::superstructure::kMagazineSolenoidB, DoubleSolenoid::Value::kForward);
    pcm_->WriteSolenoid(ports::superstructure::kShooterHoodSolenoid, false);
  }
}

}  // namespace wpilib
}  // namespace c2017
