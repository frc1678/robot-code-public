#include "o2017/wpilib/superstructure_interface.h"
#include <cmath>

namespace o2017 {

namespace wpilib {

namespace ports {

namespace superstructure {

constexpr uint32_t kHumanPlayerDeploySolenoid = 1;

constexpr uint32_t kClimberMotor = 2;

}  // namespace superstructure

}  // namespace ports

constexpr double kMaxVoltage = 12.0;
constexpr double kClimberMetersPerClick =
    muan::units::pi * (1.25 / 2.0) * muan::units::in / 512.0;

SuperstructureInterface::SuperstructureInterface(muan::wpilib::PcmWrapper* pcm)
    : input_queue_(QueueManager::GetInstance()->superstructure_input_queue()),
      output_queue_(QueueManager::GetInstance()
                        ->superstructure_output_queue()
                        ->MakeReader()),
      climber_motor_{ports::superstructure::kClimberMotor},
      climber_encoder_(14, 15),
      pcm_(pcm) {
  pcm_->CreateSolenoid(ports::superstructure::kHumanPlayerDeploySolenoid);
}

void SuperstructureInterface::ReadSensors() {
  o2017::superstructure::SuperstructureInputProto inputs;
  auto maybe_current_reader =
      QueueManager::GetInstance()->pdp_status_queue().ReadLastMessage();

  if (maybe_current_reader) {
    auto current_reader = *maybe_current_reader;
    inputs->set_climber_current(current_reader->current10() +
                                current_reader->current11() +
                                current_reader->current12());
  }
  inputs->set_climber_position(-climber_encoder_.Get() *
                               kClimberMetersPerClick);

  QueueManager::GetInstance()->superstructure_input_queue()->WriteMessage(
      inputs);
}

void SuperstructureInterface::WriteActuators() {
  auto maybe_outputs = output_queue_.ReadLastMessage();
  if (maybe_outputs) {
    auto outputs = *maybe_outputs;
    pcm_->WriteSolenoid(ports::superstructure::kHumanPlayerDeploySolenoid,
                        outputs->hp_gear_extend());
    climber_motor_.Set(muan::utils::Cap(outputs->climber_voltage(),
                                        -kMaxVoltage, kMaxVoltage) /
                       12.0);
  } else {
    pcm_->WriteSolenoid(ports::superstructure::kHumanPlayerDeploySolenoid,
                        false);
    climber_motor_.Set(0.0);
  }
}

}  // namespace wpilib
}  // namespace o2017
