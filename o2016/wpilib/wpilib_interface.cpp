#include "wpilib_interface.h"
#include "muan/units/units.h"
#include "o2016/queue_manager/queue_manager.h"

namespace o2016 {

namespace wpilib {

namespace ports {

namespace drivetrain {

//TODO(Wesley) Shifting

constexpr uint32_t kMotorLeftA = 2, kMotorLeftB = 3;
constexpr uint32_t kMotorRightA = 0, kMotorRightB = 1;

constexpr uint32_t kEncoderLeftA = 10, kEncoderLeftB = 11;
constexpr uint32_t kEncoderRightA = 12, kEncoderRightB = 13;

constexpr uint32_t kShiftingA = 0, kShiftingB = 0;

}  // drivetrain

namespace turret {

//TODO(Wesley) Index pulse

constexpr uint32_t kMotor = 6;
constexpr uint32_t kEncoderA = 14, kEncoderB = 15;
constexpr uint32_t kPotentiometer = 4;
constexpr uint32_t kIndex = 0;

}  // turret

namespace intake {

//TODO(Wesley) Index pulse, encoder, and roller motor

constexpr uint32_t kPivotMotor = 5;
constexpr uint32_t kEncoderA = 0, kEncoderB = 0;
constexpr uint32_t kIndex = 0;

constexpr uint32_t kRollerMotor = 0;

}  // intake

namespace catapult {

//TODO(Wesley) Pnumatics

constexpr uint32_t kHardStopMotor = 7;
constexpr uint32_t kHardStopPotentiometer = 6;
constexpr uint32_t kHardStopCylinder = 0;

constexpr uint32_t kScoopMotor = 4;
constexpr uint32_t kScoopPotentiometer = 5;

constexpr uint32_t kCatapultCylinderA = 0, kCatapultCylinderB = 0,
                   kCatapultCylinderC = 0, kCatapultCylinderD = 0;

}  // catapult

}  // ports

constexpr double kMaxVoltage = 12.0;

DrivetrainInterface::DrivetrainInterface(muan::wpilib::CanWrapper* can_wrapper)
    : pcm_{can_wrapper->pcm()},
      input_queue_(QueueManager::GetInstance().drivetrain_input_queue()),
      output_queue_(
          QueueManager::GetInstance().drivetrain_output_queue().MakeReader()),
      motor_left_a_{ports::drivetrain::kMotorLeftA},
      motor_left_b_{ports::drivetrain::kMotorLeftB},
      motor_right_a_{ports::drivetrain::kMotorRightA},
      motor_right_b_{ports::drivetrain::kMotorRightB},
      encoder_left_{ports::drivetrain::kEncoderLeftA,
                    ports::drivetrain::kEncoderLeftB},
      encoder_right_{ports::drivetrain::kEncoderRightA,
                     ports::drivetrain::kEncoderRightB} {
  pcm_->CreateDoubleSolenoid(ports::drivetrain::kShiftingA,
                             ports::drivetrain::kShiftingB);
}

void DrivetrainInterface::ReadSensors() {
  o2016::drivetrain::StackDrivetrainInput sensors;
  constexpr double wheel_radius = 3 * muan::units::in;
  constexpr double kMetersPerClick = M_PI * 2.0 * wheel_radius / 360.0;
  sensors->set_left_encoder(encoder_left_.Get() * kMetersPerClick);
  sensors->set_right_encoder(encoder_right_.Get() * kMetersPerClick);

  // TODO(Kyle) Use the actual gyro here
  sensors->set_gyro_angle(0.0);
  input_queue_.WriteMessage(sensors);
}

void DrivetrainInterface::WriteActuators() {
  auto outputs = output_queue_.ReadLastMessage();
  if (outputs) {
    motor_left_a_.Set(
        muan::Cap((*outputs)->left_voltage(), -kMaxVoltage, kMaxVoltage) /
        12.0);
    motor_left_b_.Set(
        muan::Cap((*outputs)->left_voltage(), -kMaxVoltage, kMaxVoltage) /
        12.0);

    motor_right_a_.Set(
        muan::Cap((*outputs)->right_voltage(), -kMaxVoltage, kMaxVoltage) /
        12.0);
    motor_right_b_.Set(
        muan::Cap((*outputs)->right_voltage(), -kMaxVoltage, kMaxVoltage) /
        12.0);

    pcm_->WriteDoubleSolenoid(
        ports::drivetrain::kShiftingA, ports::drivetrain::kShiftingB,
        (*outputs)->high_gear() ? DoubleSolenoid::Value::kForward
                                : DoubleSolenoid::Value::kReverse);
  } else {
    motor_left_a_.Set(0);
    motor_left_b_.Set(0);
    motor_right_a_.Set(0);
    motor_right_b_.Set(0);
  }
}

TurretInterface::TurretInterface()
    : input_queue_(QueueManager::GetInstance().turret_input_queue()),
      output_queue_(
          QueueManager::GetInstance().turret_output_queue().MakeReader()),
      motor_{ports::turret::kMotor},
      encoder_{ports::turret::kEncoderA, ports::turret::kEncoderB},
      potentiometer_{ports::turret::kPotentiometer},
      index_{ports::turret::kIndex} {}

void TurretInterface::WriteActuators() {
  auto outputs = output_queue_.ReadLastMessage();
  if (outputs) {
    motor_.Set(muan::Cap((*outputs)->voltage(), -kMaxVoltage, kMaxVoltage) /
               12.0);
  } else {
    motor_.Set(0.0);
  }
}

void TurretInterface::ReadSensors() {
  o2016::turret::TurretInputProto sensors;

  constexpr double kPotentiometerScaling = -360.0;
  constexpr double kPotentiometerOffset = -0.5;

  // TODO(Wesley) figure this out for realsies
  constexpr double kEncoderScaling = 1.0;

  sensors->set_encoder_position(encoder_.Get() * kEncoderScaling);
  sensors->set_pot_position(((potentiometer_.GetValue() + kPotentiometerOffset) * kPotentiometerScaling) * muan::units::deg);
  sensors->set_index_click(index_.Get());

  input_queue_.WriteMessage(sensors);
}

IntakeInterface::IntakeInterface()
    : input_queue_(QueueManager::GetInstance().intake_input_queue()),
      output_queue_(
          QueueManager::GetInstance().intake_output_queue().MakeReader()),
      motor_pivot_{ports::intake::kPivotMotor},
      motor_roller_{ports::intake::kRollerMotor},
      encoder_{ports::intake::kEncoderA, ports::intake::kEncoderB},
      index_{ports::intake::kIndex} {}

void IntakeInterface::WriteActuators() {
  auto outputs = output_queue_.ReadLastMessage();
  if (outputs) {
    motor_pivot_.Set(
        muan::Cap((*outputs)->arm_voltage(), -kMaxVoltage, kMaxVoltage) / 12.0);
    motor_roller_.Set(
        muan::Cap((*outputs)->roller_voltage(), -kMaxVoltage, kMaxVoltage) /
        12.0);
  } else {
    motor_pivot_.Set(0.0);
    motor_roller_.Set(0.0);
  }
}

void IntakeInterface::ReadSensors() {
  o2016::intake::IntakeInputProto sensors;

  constexpr double kEncoderScaling = 0.0;
  sensors->set_encoder_position(encoder_.Get() * kEncoderScaling);
  sensors->set_index_click(index_.Get());

  input_queue_.WriteMessage(sensors);
}

CatapultInterface::CatapultInterface(muan::wpilib::CanWrapper* can)
    : input_queue_(QueueManager::GetInstance().catapult_input_queue()),
      output_queue_(
          QueueManager::GetInstance().catapult_output_queue().MakeReader()),
      pcm_{can->pcm()},
      hard_stop_motor_{ports::catapult::kHardStopMotor},
      hard_stop_pot_{ports::catapult::kHardStopPotentiometer},
      scoop_motor_{ports::catapult::kScoopMotor},
      scoop_pot_{ports::catapult::kScoopPotentiometer} {
  pcm_->CreateSolenoid(ports::catapult::kCatapultCylinderA);
  pcm_->CreateSolenoid(ports::catapult::kCatapultCylinderB);
  pcm_->CreateSolenoid(ports::catapult::kCatapultCylinderC);
  pcm_->CreateSolenoid(ports::catapult::kCatapultCylinderD);

  pcm_->CreateSolenoid(ports::catapult::kHardStopCylinder);
}

void CatapultInterface::WriteActuators() {
  auto outputs = output_queue_.ReadLastMessage();
  if (outputs) {
    hard_stop_motor_.Set(
        muan::Cap((*outputs)->hardstop_output(), -kMaxVoltage, kMaxVoltage) /
        12.0);
    scoop_motor_.Set(
        muan::Cap((*outputs)->scoop_output(), -kMaxVoltage, kMaxVoltage) /
        12.0);

    pcm_->WriteSolenoid(ports::catapult::kCatapultCylinderA,
                        (*outputs)->cylinder_extend());
    pcm_->WriteSolenoid(ports::catapult::kCatapultCylinderB,
                        (*outputs)->cylinder_extend());
    pcm_->WriteSolenoid(ports::catapult::kCatapultCylinderC,
                        (*outputs)->cylinder_extend());
    pcm_->WriteSolenoid(ports::catapult::kCatapultCylinderD,
                        (*outputs)->cylinder_extend());

    pcm_->WriteSolenoid(ports::catapult::kHardStopCylinder,
                        (*outputs)->disc_brake_activate());
  } else {
    hard_stop_motor_.Set(0.0);
    scoop_motor_.Set(0.0);
    pcm_->WriteSolenoid(ports::catapult::kCatapultCylinderA, false);
    pcm_->WriteSolenoid(ports::catapult::kCatapultCylinderB, false);
    pcm_->WriteSolenoid(ports::catapult::kCatapultCylinderC, false);
    pcm_->WriteSolenoid(ports::catapult::kCatapultCylinderD, false);

    pcm_->WriteSolenoid(ports::catapult::kHardStopCylinder, false);
  }
}

void CatapultInterface::ReadSensors() {
  o2016::catapult::CatapultInputProto sensors;

  constexpr double kScoopScaling = -216.66;
  constexpr double kScoopOffset = -0.9;

  //TODO(Wesley) Test real values
  constexpr double kHardStopScaling = 1.0;
  constexpr double kHardStopOffset = 0.0;

  sensors->set_scoop_pot(((scoop_pot_.GetValue() + kScoopOffset) * kScoopScaling) * muan::units::deg);
  sensors->set_hardstop_pot((hard_stop_pot_.GetValue() + kHardStopOffset) * kHardStopScaling);

  input_queue_.WriteMessage(sensors);
}

WpilibInterface::WpilibInterface()
    : can_{&QueueManager::GetInstance().pdp_status_queue()},
      drivetrain_{&can_},
      catapult_{&can_} {
  std::thread can_thread(std::ref(can_));
  can_thread.detach();
}

void WpilibInterface::WriteActuators() {
  drivetrain_.WriteActuators();
  turret_.WriteActuators();
  intake_.WriteActuators();
  catapult_.WriteActuators();
}

void WpilibInterface::ReadSensors() {
  drivetrain_.ReadSensors();
  turret_.ReadSensors();
  intake_.ReadSensors();
  catapult_.ReadSensors();
}

}  // wpilib

}  // o2016
