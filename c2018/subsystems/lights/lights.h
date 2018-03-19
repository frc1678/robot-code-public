#ifndef C2018_SUBSYSTEMS_LIGHTS_LIGHTS_H_
#define C2018_SUBSYSTEMS_LIGHTS_LIGHTS_H_

#include "c2018/subsystems/lights/queue_types.h"
#include "c2018/subsystems/score_subsystem/queue_types.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/queue_types.h"

namespace c2018 {
namespace lights {

constexpr double kBlinkHz = 3;
constexpr double kFlashLength = 1.5;

class Lights {
 public:
  Lights();
  void Update();

 private:
  double flash_time_ = 0.0;  // light flashes if this is less than 2.0
  // flash_time_ is how long the robot has had the cube for
  bool had_cube_ = false;
  bool FlashLights();

  LightsOutputQueue* output_queue_;
  c2018::score_subsystem::ScoreSubsystemStatusQueue::QueueReader status_reader_;
};

}  // namespace lights
}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_LIGHTS_LIGHTS_H_
