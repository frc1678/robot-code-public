#include "wpilib_interface.h"
#include "muan/units/units.h"
#include "o2016/queue_manager/queue_manager.h"

namespace o2016 {

namespace wpilib {

namespace ports {

namespace drivetrain {

constexpr uint32_t kMotorLeftA = 0, kMotorLeftB = 0;
constexpr uint32_t kMotorRightA = 0, kMotorRightB = 0;

constexpr uint32_t kEncoderLeftA = 0, kEncoderLeftB = 0;
constexpr uint32_t kEncoderRightA = 0, kEncoderRightB = 0;

constexpr uint32_t kShiftingA = 0, kShiftingB = 0;

}  // drivetrain

namespace turret {

constexpr uint32_t kMotor = 0;
constexpr uint32_t kEncoderA = 0, kEncoderB = 0;
constexpr uint32_t kPotentiometer = 0;
constexpr uint32_t kIndex = 0;

}  // turret

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

  // TODO(Kyle) figure these out for reals
  constexpr double kPotentiometerScaling = 1.0;
  constexpr double kEncoderScaling = 1.0;

  sensors->set_encoder_position(encoder_.Get() * kEncoderScaling);
  sensors->set_pot_position(potentiometer_.GetValue() * kPotentiometerScaling);
  sensors->set_index_click(index_.Get());

  input_queue_.WriteMessage(sensors);
}

WpilibInterface::WpilibInterface()
    : can_{&QueueManager::GetInstance().pdp_status_queue()},
      drivetrain_{&can_} {
  std::thread can_thread(std::ref(can_));
  can_thread.detach();
}

void WpilibInterface::WriteActuators() {
  drivetrain_.WriteActuators();
  turret_.WriteActuators();
}

void WpilibInterface::ReadSensors() {
  drivetrain_.ReadSensors();
  turret_.ReadSensors();
}

}  // wpilib

}  // o2016
