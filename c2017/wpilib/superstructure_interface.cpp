#include "c2017/wpilib/superstructure_interface.h"

namespace c2017 {

namespace wpilib {

namespace ports {

namespace superstructure {

// Motor ports
constexpr uint32_t kShooterMotor = 7;
constexpr uint32_t kAccelMotor = 2;

constexpr uint32_t kUpperConveyorMotor = 5;
constexpr uint32_t kSideConveyorMotor = 4;

constexpr uint32_t kBallIntakeMotor = 3;

constexpr uint32_t kGearIntakeMotor = 6;

// Sensor ports
constexpr uint32_t kShooterEncoderA = 20, kShooterEncoderB = 21;
constexpr uint32_t kAccelEncoderA = 16, kAccelEncoderB = 17;

// Solenoid ports
constexpr uint32_t kBallIntakeSolenoid = 7;
constexpr uint32_t kGroundGearIntakeSolenoid = 4;
constexpr uint32_t kMagazineSolenoid = 6;
constexpr uint32_t kClimberSolenoidA = 0;
constexpr uint32_t kClimberSolenoidB = 1;

// Other
constexpr double kMaxVoltage = 12;

}  // namespace superstructure

}  // namespace ports

SuperStructureInterface::SuperStructureInterface(muan::wpilib::CanWrapper* can_wrapper)
    : output_queue_(QueueManager::GetInstance().superstructure_output_queue().MakeReader()),
      shooter_motor_{ports::superstructure::kShooterMotor},
      accel_motor_{ports::superstructure::kAccelMotor},
      upper_conveyor_motor_{ports::superstructure::kUpperConveyorMotor},
      side_conveyor_motor_{ports::superstructure::kSideConveyorMotor},
      ball_intake_motor_{ports::superstructure::kBallIntakeMotor},
      gear_intake_motor_{ports::superstructure::kGearIntakeMotor},
      shooter_encoder_{ports::superstructure::kShooterEncoderA, ports::superstructure::kShooterEncoderB},
      accel_encoder_{ports::superstructure::kAccelEncoderA, ports::superstructure::kAccelEncoderB},
      pcm_{can_wrapper->pcm()} {
  pcm_->CreateSolenoid(ports::superstructure::kBallIntakeSolenoid);
  pcm_->CreateSolenoid(ports::superstructure::kGroundGearIntakeSolenoid);
  pcm_->CreateDoubleSolenoid(ports::superstructure::kClimberSolenoidA,
                             ports::superstructure::kClimberSolenoidB);
  pcm_->CreateSolenoid(ports::superstructure::kMagazineSolenoid);
}

void SuperStructureInterface::ReadSensors() {
  c2017::shooter::ShooterInputProto shooter_sensors;
  c2017::climber::ClimberInputProto climber_sensors;

  c2017::ground_gear_intake::GroundGearIntakeInputProto ground_gear_sensors;

  constexpr double kShooterRadiansPerClick = M_PI * 2 / 512.0;
  constexpr double kAccelRadiansPerClick = M_PI * 2 / 512.0;
  constexpr double kClimberMetersPerClick = M_PI * 2 / 512.0 / 25.6 * 0.032;

  shooter_sensors->set_shooter_encoder_position(shooter_encoder_.Get() * kShooterRadiansPerClick);
  shooter_sensors->set_accelerator_encoder_position(-accel_encoder_.Get() * kAccelRadiansPerClick);
  climber_sensors->set_position(accel_encoder_.Get() * kClimberMetersPerClick);

  auto current_reader = QueueManager::GetInstance().pdp_status_queue().MakeReader().ReadLastMessage();

  if (current_reader) {
    climber_sensors->set_current((*current_reader)->current14());
    ground_gear_sensors->set_current((*current_reader)->current11());
  }

  QueueManager::GetInstance().shooter_input_queue().WriteMessage(shooter_sensors);
  QueueManager::GetInstance().climber_input_queue().WriteMessage(climber_sensors);
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

    accel_motor_.Set(-muan::utils::Cap(-(*outputs)->accelerator_voltage(),
                                       -ports::superstructure::kMaxVoltage,
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
    pcm_->WriteSolenoid(ports::superstructure::kGroundGearIntakeSolenoid, (*outputs)->ground_gear_down());
    pcm_->WriteDoubleSolenoid(
        ports::superstructure::kClimberSolenoidA, ports::superstructure::kClimberSolenoidB,
        !(*outputs)->climber_engaged() ? DoubleSolenoid::Value::kReverse : DoubleSolenoid::Value::kForward);
    pcm_->WriteSolenoid(ports::superstructure::kMagazineSolenoid, (*outputs)->magazine_open());

  } else {
    shooter_motor_.Set(0);
    upper_conveyor_motor_.Set(0);
    side_conveyor_motor_.Set(0);
    ball_intake_motor_.Set(0);
    gear_intake_motor_.Set(0);

    pcm_->WriteSolenoid(ports::superstructure::kBallIntakeSolenoid, false);
    pcm_->WriteSolenoid(ports::superstructure::kGroundGearIntakeSolenoid, false);
    pcm_->WriteDoubleSolenoid(ports::superstructure::kClimberSolenoidA,
                              ports::superstructure::kClimberSolenoidB, DoubleSolenoid::Value::kForward);
    pcm_->WriteSolenoid(ports::superstructure::kMagazineSolenoid, false);
  }
}

}  // namespace wpilib
}  // namespace c2017
