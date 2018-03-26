#ifndef C2018_SUBSYSTEMS_LIGHTS_LIGHTS_H_
#define C2018_SUBSYSTEMS_LIGHTS_LIGHTS_H_

#include "c2018/subsystems/lights/queue_types.h"
#include "c2018/subsystems/score_subsystem/queue_types.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/queue_types.h"

namespace c2018 {
namespace lights {

constexpr double kBlinkHzFast = 10.0;
constexpr double kBlinkHzSlow = 4.0;
constexpr int kFlashTicks = 400;

class Lights {
 public:
  Lights();
  void Update();

 private:
  int flash_pickup_ticks_left_ = 0;
  int flash_request_ticks_gone_ = 0;

  // flash_time_ is how long the robot has had the cube for
  bool had_cube_ = false;
  bool FlashLights(double hz, int ticks_gone);

  LightsOutputQueue* output_queue_;
  score_subsystem::ScoreSubsystemStatusQueue::QueueReader status_reader_;
  LightsGoalQueue::QueueReader goal_reader_;
};

}  // namespace lights
}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_LIGHTS_LIGHTS_H_
