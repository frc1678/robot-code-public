#include "c2017/wpilib/superstructure_interface.h"

namespace c2017 {

namespace wpilib {

namespace ports {

namespace superstructure {

//TODO: Kelly figure out the correct ports for everything
// Motor ports
constexpr uint32_t kShooterMotor = 2;

constexpr uint32_t kTriggerMotor = 5;

constexpr uint32_t kBrushMotor = 4;

constexpr uint32_t kBallIntakeMotor = 3;

constexpr uint32_t kGearIntakeMotor = 6;

// Sensor ports
constexpr uint32_t kShooterEncoderA = 14, kShooterEncoderB = 15;
constexpr uint32_t kTriggerEncoderA = 16, kTriggerEncoderB = 17;

// Solenoid ports
constexpr uint32_t kBallIntakeSolenoid = 1;
constexpr uint32_t kGroundGearIntakeSolenoid = 2;
constexpr uint32_t kGearShutterSolenoid = 3;
constexpr uint32_t kHpGearIntakeSolenoid = 4;
constexpr uint32_t kMagazineSolenoid = 5;

// Other
constexpr double kMaxVoltage = 12;

} // superstrucute

} // ports

SuperStructureInterface::SuperStructureInterface(muan::wpilib::CanWrapper* can_wrapper)
    : output_queue_(QueueManager::GetInstance().superstructure_output_queue().MakeReader()),
      shooter_motor_{ports::superstructure::kShooterMotor},
      trigger_motor_{ports::superstructure::kTriggerMotor},
      brush_motor_{ports::superstructure::kBrushMotor},
      ball_intake_motor_{ports::superstructure::kBallIntakeMotor},
      gear_intake_motor_{ports::superstructure::kGearIntakeMotor},
      shooter_encoder_{ports::superstructure::kShooterEncoderA, ports::superstructure::kShooterEncoderB},
      trigger_encoder_{ports::superstructure::kTriggerEncoderA, ports::superstructure::kTriggerEncoderB},
      pcm_{can_wrapper->pcm()} {

  pcm_->CreateSolenoid(ports::superstructure::kBallIntakeSolenoid);
  pcm_->CreateSolenoid(ports::superstructure::kGroundGearIntakeSolenoid);
  pcm_->CreateSolenoid(ports::superstructure::kGearShutterSolenoid);
  pcm_->CreateSolenoid(ports::superstructure::kHpGearIntakeSolenoid);
  pcm_->CreateSolenoid(ports::superstructure::kMagazineSolenoid);

}

void SuperStructureInterface::ReadSensors() {
  c2017::shooter::ShooterInputProto shooter_sensors;
  c2017::trigger::TriggerInputProto trigger_sensors;
  c2017::climber::ClimberInputProto climber_sensors;

  c2017::magazine::MagazineInputProto magazine_sensors;
  c2017::ground_gear_intake::GroundGearIntakeInputProto ground_gear_sensors;

  constexpr double kRadiansPerClick = M_PI * 2  / 512.0;

  shooter_sensors->set_encoder_position(shooter_encoder_.Get() * kRadiansPerClick);
  trigger_sensors->set_encoder_position(trigger_encoder_.Get() * kRadiansPerClick);
  climber_sensors->set_position(shooter_encoder_.Get() * kRadiansPerClick);

  auto current_reader = QueueManager::GetInstance().pdp_status_queue().MakeReader().ReadLastMessage();

  if (current_reader) {
    // Place holder for now
    // TODO: Kelly figure out what exactly needs current inputs and what ports it is in
    // Ground gear intake, climber
    ground_gear_sensors->set_current((*current_reader)->current4());
    climber_sensors->set_current((*current_reader)->current5());
    magazine_sensors->set_conveyor_current((*current_reader)->current6());
  }

  shooter_input_queue_.WriteMessage(shooter_sensors);
  trigger_input_queue_.WriteMessage(trigger_sensors);
  climber_input_queue_.WriteMessage(climber_sensors);

  magazine_input_queue_.WriteMessage(magazine_sensors);
  ground_gear_input_queue_.WriteMessage(ground_gear_sensors);
}

void SuperStructureInterface::WriteActuators() {
  auto outputs = output_queue_.ReadLastMessage();
  auto current_reader = QueueManager::GetInstance().pdp_status_queue().MakeReader().ReadLastMessage();
  if (outputs) {
    // Shooter motors
    shooter_motor_.Set(-muan::utils::Cap((*outputs)->shooter_voltage(), -ports::superstructure::kMaxVoltage, ports::superstructure::kMaxVoltage) / 12.0);

    // Trigger motor
    trigger_motor_.Set(
        -muan::utils::Cap((*outputs)->trigger_voltage(), -ports::superstructure::kMaxVoltage, ports::superstructure::kMaxVoltage) /
        12.0);

    // Brush motor
    brush_motor_.Set(
        -muan::utils::Cap((*outputs)->brush_voltage(), -ports::superstructure::kMaxVoltage, ports::superstructure::kMaxVoltage) /
        12.0);

    // Main ball intake motors
    ball_intake_motor_.Set(
        -muan::utils::Cap((*outputs)->main_roller_voltage(), -ports::superstructure::kMaxVoltage, ports::superstructure::kMaxVoltage) /
        12.0);

    // Ground gear intake motor
    gear_intake_motor_.Set(
        -muan::utils::Cap((*outputs)->trigger_voltage(), -ports::superstructure::kMaxVoltage, ports::superstructure::kMaxVoltage) /
        12.0);

    // Solenoids
    pcm_->WriteSolenoid(ports::superstructure::kBallIntakeSolenoid, (*outputs)->ball_intake_down());
    pcm_->WriteSolenoid(ports::superstructure::kGroundGearIntakeSolenoid, (*outputs)->ground_gear_down());
    pcm_->WriteSolenoid(ports::superstructure::kGearShutterSolenoid, (*outputs)->gear_shutter_open());
    pcm_->WriteSolenoid(ports::superstructure::kHpGearIntakeSolenoid, (*outputs)->hp_gear_open());
    pcm_->WriteSolenoid(ports::superstructure::kMagazineSolenoid, (*outputs)->magazine_open());

  } else {
    shooter_motor_.Set(0);
    trigger_motor_.Set(0);
    brush_motor_.Set(0);
    ball_intake_motor_.Set(0);
    gear_intake_motor_.Set(0);

    pcm_->WriteSolenoid(ports::superstructure::kBallIntakeSolenoid, false);
    pcm_->WriteSolenoid(ports::superstructure::kGroundGearIntakeSolenoid, false);
    pcm_->WriteSolenoid(ports::superstructure::kGearShutterSolenoid, false);
    pcm_->WriteSolenoid(ports::superstructure::kHpGearIntakeSolenoid, false);
    pcm_->WriteSolenoid(ports::superstructure::kMagazineSolenoid, false);
  }
}



} // wpilib

} // c2017
