#include "testbench/wpilib/wpilib_interface.h"
#include "muan/units/units.h"
#include "testbench/queue_manager/queue_manager.h"

namespace testbench {

namespace wpilib {

namespace ports {

namespace drivetrain {

// TODO(Wesley) Shifting

constexpr uint32_t kMotorLeft = 1;
constexpr uint32_t kMotorRight = 0;

constexpr uint32_t kEncoderLeftA = 12, kEncoderLeftB = 13;
constexpr uint32_t kEncoderRightA = 10, kEncoderRightB = 11;

constexpr uint32_t kShifting = 7;

}  // namespace drivetrain

}  // namespace ports

constexpr double kMaxVoltage = 12;

DrivetrainInterface::DrivetrainInterface(muan::wpilib::CanWrapper* can_wrapper)
    : pcm_{can_wrapper->pcm()},
      input_queue_(QueueManager::GetInstance()->drivetrain_input_queue()),
      output_queue_(
          QueueManager::GetInstance()->drivetrain_output_queue()->MakeReader()),
      motor_left_{ports::drivetrain::kMotorLeft},
      motor_right_{ports::drivetrain::kMotorRight},
      encoder_left_{ports::drivetrain::kEncoderLeftA,
                    ports::drivetrain::kEncoderLeftB},
      encoder_right_{ports::drivetrain::kEncoderRightA,
                     ports::drivetrain::kEncoderRightB} {
  pcm_->CreateSolenoid(ports::drivetrain::kShifting);
}

void DrivetrainInterface::ReadSensors() {
  frc971::control_loops::drivetrain::InputProto sensors;
  constexpr double wheel_radius = 3 * muan::units::in;
  constexpr double kMetersPerClick = M_PI * 2.0 * wheel_radius / 360.0;
  sensors->set_left_encoder(encoder_left_.Get() * kMetersPerClick);
  sensors->set_right_encoder(-encoder_right_.Get() * kMetersPerClick);

  input_queue_->WriteMessage(sensors);
}

void DrivetrainInterface::WriteActuators() {
  auto outputs = output_queue_.ReadLastMessage();
  if (outputs) {
    motor_left_.Set(-muan::utils::Cap((*outputs)->left_voltage(), -kMaxVoltage,
                                      kMaxVoltage) /
                    12.0);

    motor_right_.Set(muan::utils::Cap((*outputs)->right_voltage(), -kMaxVoltage,
                                      kMaxVoltage) /
                     12.0);

    // TODO(Wesley) Verify high gear/low gear
    pcm_->WriteSolenoid(ports::drivetrain::kShifting, (*outputs)->high_gear());
  } else {
    motor_left_.Set(0);
    motor_left_.Set(0);
  }
}

WpilibInterface::WpilibInterface()
    : can_{QueueManager::GetInstance()->pdp_status_queue()},
      gyro_{QueueManager::GetInstance()->gyro_queue(),
            QueueManager::GetInstance()->driver_station_queue(), 45},
      drivetrain_{&can_} {
  std::thread can_thread(std::ref(can_));
  can_thread.detach();

  std::thread gyro_thread(std::ref(gyro_));
  gyro_thread.detach();
}

void WpilibInterface::WriteActuators() { drivetrain_.WriteActuators(); }

void WpilibInterface::ReadSensors() { drivetrain_.ReadSensors(); }

}  // namespace wpilib

}  // namespace testbench
