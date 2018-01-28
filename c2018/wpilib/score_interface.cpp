#include "c2018/wpilib/score_interface.h"

#include <algorithm>

#include "muan/utils/math_utils.h"

namespace c2018 {
namespace wpilib {

constexpr double kPitchRadius = (1. + (1. / 16.)) * 0.0254;

constexpr double kElevatorSensorRatio = 2.14;
constexpr double kWristSensorRatio = 5.14;

constexpr uint32_t kElevatorMotor = 4;
constexpr uint32_t kIntakeMotor = 2;
constexpr uint32_t kWristMotor = 3;

constexpr uint32_t kElevatorEncoderA = 14;
constexpr uint32_t kElevatorEncoderB = 15;
constexpr uint32_t kElevatorEncoderIndex = 4;

constexpr uint32_t kWristEncoderA = 18;
constexpr uint32_t kWristEncoderB = 19;
constexpr uint32_t kWristEncoderIndex = 6;

constexpr uint32_t kIntakeSolenoid = 2;

constexpr uint32_t kWristPotentiometer = 0;

constexpr uint32_t kElevatorHall = 0;
constexpr uint32_t kWristHall = 3;
constexpr uint32_t kCubeProxy = 1;

constexpr double kMaxVoltage = 12;

ScoreSubsystemInterface::ScoreSubsystemInterface(
    muan::wpilib::CanWrapper* can_wrapper)
    : input_queue_(QueueManager<ScoreSubsystemInputProto>::Fetch()),
      output_reader_(
          QueueManager<ScoreSubsystemOutputProto>::Fetch()->MakeReader()),
      pdp_reader_(
          QueueManager<muan::wpilib::PdpMessage>::Fetch()->MakeReader()),
      elevator_{kElevatorMotor},
      wrist_{kWristMotor},
      roller_{kIntakeMotor},
      elevator_encoder_{kElevatorEncoderA, kElevatorEncoderB},
      wrist_encoder_{kWristEncoderA, kWristEncoderB},
      has_cube_{kCubeProxy},
      elevator_hall_{kElevatorHall},
      wrist_hall_{kWristHall},
      pcm_{can_wrapper->pcm()} {
  pcm_->CreateSolenoid(kIntakeSolenoid);
}

void ScoreSubsystemInterface::ReadSensors() {
  ScoreSubsystemInputProto sensors;
  sensors->set_elevator_encoder(elevator_encoder_.Get() * kPitchRadius *
                                (2 * M_PI) / 512 / kElevatorSensorRatio);
  sensors->set_wrist_encoder(wrist_encoder_.Get() * (2 * M_PI) / 512 /
                             kWristSensorRatio);
  // These numbers come from the status to outpur ratios for the encoders.
  sensors->set_elevator_hall(elevator_hall_.Get());
  sensors->set_wrist_hall(wrist_hall_.Get());
  sensors->set_has_cube(has_cube_.Get());

  muan::wpilib::PdpMessage pdp_data;
  if (pdp_reader_.ReadLastMessage(&pdp_data)) {
    sensors->set_intake_current(
        std::max(pdp_data->current5(), pdp_data->current6()));
  }
}

void ScoreSubsystemInterface::WriteActuators() {
  ScoreSubsystemOutputProto outputs;
  if (output_reader_.ReadLastMessage(&outputs)) {
    elevator_.Set(muan::utils::Cap(outputs->elevator_voltage(), -kMaxVoltage,
                                   kMaxVoltage));
    wrist_.Set(
        muan::utils::Cap(outputs->wrist_voltage(), -kMaxVoltage, kMaxVoltage));
    roller_.Set(
        muan::utils::Cap(outputs->intake_voltage(), -kMaxVoltage, kMaxVoltage));
    pcm_->WriteSolenoid(kIntakeSolenoid, outputs->claw_pinch());
  } else {
    elevator_.Set(0);
    wrist_.Set(0);
    roller_.Set(0);
    pcm_->WriteSolenoid(kIntakeSolenoid, false);
  }
}

}  // namespace wpilib
}  // namespace c2018
