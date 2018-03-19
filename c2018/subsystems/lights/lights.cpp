#include "c2018/subsystems/lights/lights.h"

using muan::queues::QueueManager;

namespace c2018 {
namespace lights {

Lights::Lights()
    : output_queue_{QueueManager<LightsOutputProto>::Fetch()},
      status_reader_{
          QueueManager<score_subsystem::ScoreSubsystemStatusProto>::Fetch()
              ->MakeReader()} {}

bool Lights::FlashLights() {
  const double cycle_period = 1 / kBlinkHz;
  return fmod(flash_time_, cycle_period) < cycle_period / 2;
}

void Lights::Update() {
  c2018::score_subsystem::ScoreSubsystemStatusProto status_proto;
  LightsOutputProto output;
  status_reader_.ReadLastMessage(&status_proto);
  bool on = false;

  if (flash_time_ < kFlashLength) {  // stop flashing after timer is up
    on = FlashLights();
  }
  if (status_proto->has_cube() && !had_cube_) {
    // if it just got a cube, start flashing
    flash_time_ = 0.0;
  }

  flash_time_ += 0.005;
  had_cube_ = status_proto->has_cube();

  output->set_on(on);
  output_queue_->WriteMessage(output);
}

}  // namespace lights
}  // namespace c2018
