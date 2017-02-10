#include "c2017/subsystems/lights/lights.h"

namespace c2017 {

namespace lights {
void Lights::Update() {
  auto intake_group_goal_queue = QueueManager::GetInstance().intake_group_goal_queue().ReadLastMessage();
  auto drivetrain_status_queue = QueueManager::GetInstance().drivetrain_status_queue()->ReadLastMessage();
  auto gyro_status_queue = QueueManager::GetInstance().gyro_queue()->ReadLastMessage();
  auto vision_status = QueueManager::GetInstance().vision_status_queue().ReadLastMessage();

  if (gyro_status_queue) {
    if (gyro_status_queue.value()->calibration_time_left() <= 0) {
      if (intake_group_goal_queue) {
        switch (intake_group_goal_queue.value()->hp_load_type()) {
          case c2017::intake_group::HpLoadType::HP_LOAD_NONE:
            light_color_ = VisionAlignment();
            break;
          case c2017::intake_group::HpLoadType::HP_LOAD_BALLS:
            light_color_ = LightColor::YELLOW;
            break;
          case c2017::intake_group::HpLoadType::HP_LOAD_GEAR:
            light_color_ = LightColor::PINK;
            break;
          case c2017::intake_group::HpLoadType::HP_LOAD_BOTH:
            light_color_ = FlashLights(LightColor::YELLOW, LightColor::PINK, false);
            break;
          default:
            light_color_ = FlashLights(LightColor::OFF, LightColor::TEAL, false);
        }
      } else {
        light_color_ = FlashLights(LightColor::OFF, LightColor::TEAL, false);
      }
      // add the auto routines, not nessecarily here, but somewhere.
    } else {
      light_color_ = LightColor::RED;
    }
  } else {
    light_color_ = FlashLights(LightColor::OFF, LightColor::TEAL, false);
  }

  if (vision_status) {
    light_color_ = FlashLights(light_color_, light_color_, !vision_status.value()->has_connection());
  } else {
    light_color_ = FlashLights(light_color_, light_color_, true);
  }

  LightsOutputProto output;

  output->set_red(is_red());
  output->set_green(is_green());
  output->set_blue(is_blue());

  QueueManager::GetInstance().lights_output_queue().WriteMessage(output);
}

LightColor Lights::VisionAlignment() {
  auto vision_status = QueueManager::GetInstance().vision_status_queue().ReadLastMessage();
  if (vision_status) {
    if (!vision_status.value()->target_found()) {
      return LightColor::RED;
    } else if (!vision_status.value()->aligned()) {
      return LightColor::YELLOW;
    } else {
      return LightColor::GREEN;
    }
  }
  return LightColor::WHITE;
}
bool Lights::is_red() const {
  return (light_color_ == LightColor::RED || light_color_ == LightColor::YELLOW ||
          light_color_ == LightColor::WHITE || light_color_ == LightColor::PINK);
}

LightColor Lights::FlashLights(LightColor color_one, LightColor color_two, bool off_between) {
  double now =
      std::chrono::duration<double>(aos::monotonic_clock::now() - aos::monotonic_clock::epoch()).count();
  auto color = (static_cast<int>(now) % 2) ? color_one : color_two;
  if (off_between && fmod(now, 0.5) < 0.25) color = LightColor::OFF;
  return color;
}

bool Lights::is_green() const {
  return (light_color_ == LightColor::GREEN || light_color_ == LightColor::TEAL ||
          light_color_ == LightColor::YELLOW || light_color_ == LightColor::WHITE);
}

bool Lights::is_blue() const {
  return (light_color_ == LightColor::BLUE || light_color_ == LightColor::TEAL ||
          light_color_ == LightColor::PINK || light_color_ == LightColor::WHITE);
}

}  // namespace lights

}  // namespace c2017
