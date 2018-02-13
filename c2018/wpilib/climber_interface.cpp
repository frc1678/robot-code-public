#include "c2018/wpilib/climber_interface.h"

namespace c2018 {
namespace wpilib {

constexpr uint32_t kWinchMotor = 5;

constexpr uint32_t kWinchEncoderA = 16;
constexpr uint32_t kWinchEncoderB = 17;
constexpr uint32_t kWinchEncoderIndex = 5;

constexpr uint32_t kBatterSolenoid = 2;

constexpr double kMaxVoltage = 12;

ClimberInterface::ClimberInterface(muan::wpilib::CanWrapper* can_wrapper)
    : input_queue_(QueueManager<ClimberInputProto>::Fetch()),
      output_reader_(QueueManager<ClimberOutputProto>::Fetch()->MakeReader()),
      pdp_reader_(
          QueueManager<muan::wpilib::PdpMessage>::Fetch()->MakeReader()),
      winch_{kWinchMotor},
      winch_encoder_{kWinchEncoderA, kWinchEncoderB},
      pcm_{can_wrapper->pcm()} {
  pcm_->CreateSolenoid(kBatterSolenoid);
}

void ClimberInterface::ReadSensors() {
  constexpr double kWinchEncoderRatio = 1.0 / 512.0;

  ClimberInputProto sensors;
  sensors->set_position(winch_encoder_.Get() * kWinchEncoderRatio);

  muan::wpilib::PdpMessage pdp_data;
  if (pdp_reader_.ReadLastMessage(&pdp_data)) {
    sensors->set_current(std::max(pdp_data->current5(), pdp_data->current6()));
  }
}

void ClimberInterface::WriteActuators() {
  ClimberOutputProto outputs;
  if (output_reader_.ReadLastMessage(&outputs)) {
    winch_.Set(muan::utils::Cap(outputs->voltage(), -kMaxVoltage, kMaxVoltage) /
               12.0);
    pcm_->WriteSolenoid(kBatterSolenoid, outputs->release_solenoid());
  } else {
    winch_.Set(0);
    pcm_->WriteSolenoid(kBatterSolenoid, false);
    LOG_P("No output queue");
  }
}

}  // namespace wpilib
}  // namespace c2018
