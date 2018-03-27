#include "c2018/subsystems/lights/lights.h"

using muan::queues::QueueManager;

namespace c2018 {
namespace lights {

Lights::Lights()
    : output_queue_{QueueManager<LightsOutputProto>::Fetch()},
      status_reader_{
          QueueManager<score_subsystem::ScoreSubsystemStatusProto>::Fetch()
              ->MakeReader()},
      goal_reader_{QueueManager<LightsGoalProto>::Fetch()->MakeReader()} {}

bool Lights::FlashLights(double hz, int ticks_gone) {
  const int cycle_period = static_cast<int>(200.0 / hz);
  return (ticks_gone / cycle_period) % 2 == 0;
}

void Lights::Update() {
  c2018::score_subsystem::ScoreSubsystemStatusProto status_proto;
  LightsOutputProto output;
  status_reader_.ReadLastMessage(&status_proto);

  LightsGoalProto goal;
  goal_reader_.ReadLastMessage(&goal);

  bool on = false;

  if (status_proto->has_cube() && !had_cube_) {
    flash_pickup_ticks_left_ = kFlashTicks;
  }

  if (goal->ask_for_cube()) {
    flash_request_ticks_gone_++;
    on = FlashLights(kBlinkHzFast, flash_request_ticks_gone_);
  } else {
    flash_request_ticks_gone_ = 0;
    if (flash_pickup_ticks_left_ > 0) {
      on = FlashLights(kBlinkHzSlow, kFlashTicks - flash_pickup_ticks_left_);
      flash_pickup_ticks_left_--;
    }
  }

  had_cube_ = status_proto->has_cube();

  output->set_on(on);
  output_queue_->WriteMessage(output);
}

}  // namespace lights
}  // namespace c2018
