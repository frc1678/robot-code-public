#include "c2018/wpilib/drivetrain_interface.h"

namespace c2018 {
namespace wpilib {

namespace constants {

constexpr uint32_t kMotorLeft = 1;
constexpr uint32_t kMotorRight = 0;

constexpr uint32_t kEncoderLeftA = 12, kEncoderLeftB = 13;
constexpr uint32_t kEncoderRightA = 10, kEncoderRightB = 11;

constexpr uint32_t kShifter = 0;

constexpr double kMaxVoltage = 12;

}  // namespace constants

DrivetrainInterface::DrivetrainInterface(muan::wpilib::CanWrapper* can_wrapper)
    : input_queue_(QueueManager<DrivetrainInputProto>::Fetch()),
      output_queue_(QueueManager<DrivetrainOutputProto>::Fetch()->MakeReader()),
      motor_left_{constants::kMotorLeft},
      motor_right_{constants::kMotorRight},
      encoder_left_{constants::kEncoderLeftA, constants::kEncoderLeftB},
      encoder_right_{constants::kEncoderRightA, constants::kEncoderRightB},
      pcm_{can_wrapper->pcm()} {
  pcm_->CreateSolenoid(constants::kShifter);
}

void DrivetrainInterface::ReadSensors() {
  frc971::control_loops::drivetrain::InputProto sensors;
  constexpr double wheel_radius = (6.0 / 2) * muan::units::in;
  constexpr double kMetersPerClick = M_PI * 2.0 * wheel_radius / 512.0;
  sensors->set_left_encoder(encoder_left_.Get() * kMetersPerClick);
  sensors->set_right_encoder(-encoder_right_.Get() * kMetersPerClick);

  input_queue_->WriteMessage(sensors);
}

void DrivetrainInterface::WriteActuators() {
  auto outputs = output_queue_.ReadLastMessage();
  if (outputs) {
    motor_left_.Set(
        muan::utils::Cap((*outputs)->left_voltage(), -constants::kMaxVoltage, constants::kMaxVoltage) / 12.0);

    motor_right_.Set(
        -muan::utils::Cap((*outputs)->right_voltage(), -constants::kMaxVoltage, constants::kMaxVoltage) /
        12.0);

    pcm_->WriteSolenoid(constants::kShifter, !(*outputs)->high_gear());
  } else {
    motor_left_.Set(0);
    motor_right_.Set(0);
  }
}

}  // namespace wpilib
}  // namespace c2018
