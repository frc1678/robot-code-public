#include "c2017/subsystems/lights/lights.h"

namespace c2017 {

namespace lights {
void Lights::Update() {
  auto intake_group_goal_queue = QueueManager::GetInstance().intake_group_goal_queue().ReadLastMessage();
  auto climber_status_queue = QueueManager::GetInstance().climber_status_queue().ReadLastMessage();
  auto drivetrain_status_queue = QueueManager::GetInstance().drivetrain_status_queue()->ReadLastMessage();
  auto gyro_status_queue = QueueManager::GetInstance().gyro_queue()->ReadLastMessage();
  auto vision_status = QueueManager::GetInstance().vision_status_queue().ReadLastMessage();
  auto ds_status = QueueManager::GetInstance().driver_station_queue()->ReadLastMessage();
  auto ground_gear_status = QueueManager::GetInstance().ground_gear_status_queue().ReadLastMessage();
  auto webdash_queue = QueueManager::GetInstance().webdash_queue().ReadLastMessage();

  if (!calibrated_ && gyro_status_queue) {
    light_color_ = LightColor::RED;
  } else if (calibrated_ && !auto_running_) {
    light_color_ = LightColor::BLUE;
  } else if (calibrated_ && auto_running_) {
    if (webdash_queue) {
      switch (webdash_queue.value()->auto_mode()) {
        case c2017::webdash::WebDash::NONE:
          light_color_ = LightColor::PINK;
          break;
        case c2017::webdash::WebDash::BLUE_HELLA_KPA:
          light_color_ = FlashLights(LightColor::BLUE, LightColor::PINK, false);
          break;
        case c2017::webdash::WebDash::BLUE_HELLA_KPA_PLUS_GEAR:
          light_color_ = FlashLights(LightColor::BLUE, LightColor::TEAL, false);
          break;
        case c2017::webdash::WebDash::RED_HELLA_KPA:
          light_color_ = FlashLights(LightColor::RED, LightColor::PINK, false);
          break;
        case c2017::webdash::WebDash::RED_HELLA_KPA_PLUS_GEAR:
          light_color_ = FlashLights(LightColor::RED, LightColor::TEAL, false);
          break;
        case c2017::webdash::WebDash::TWO_GEAR:
          light_color_ = LightColor::WHITE;
          break;
        default:
          light_color_ = FlashLights(LightColor::OFF, LightColor::PINK, false);
          break;
      }
    } else {
      light_color_ = FlashLights(LightColor::OFF, LightColor::PINK, false);
    }
  }

  if (gyro_status_queue) {
    if (gyro_status_queue.value()->calibration_time_left() <= 0) {
      calibrated_ = true;
    }
  }

  if (ds_status) {
    if (ds_status.value()->mode() == RobotMode::TELEOP) {
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
      // Climbing lights
      if (climber_status_queue) {
        if (climber_status_queue.value()->climber_state() == c2017::climber::State::CLIMBING) {
          light_color_ = LightColor::PINK;
        } else if (climber_status_queue.value()->climber_state() == c2017::climber::State::SLOWING) {
          light_color_ = LightColor::YELLOW;
        } else if (climber_status_queue.value()->climber_state() == c2017::climber::State::AT_TOP) {
          light_color_ = LightColor::GREEN;
        }
      }

      if (ground_gear_status) {
        if (ground_gear_status.value()->current_state() == c2017::ground_gear_intake::State::INTAKING) {
          light_color_ = LightColor::PINK;
        } else if (ground_gear_status.value()->current_state() ==
                       c2017::ground_gear_intake::State::CARRYING ||
                   ground_gear_status.value()->current_state() ==
                       c2017::ground_gear_intake::State::PICKING_UP) {
          light_color_ = LightColor::GREEN;
        }
      }
    } else if (ds_status.value()->mode() == RobotMode::DISABLED) {
      auto_running_ = true;
    }
  } else if (!calibrated_) {
    light_color_ = LightColor::RED;
  } else if (calibrated_) {
    light_color_ = LightColor::BLUE;
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
    } else if (vision_status.value()->aligned()) {
      return LightColor::GREEN;
    } else {
      return LightColor::YELLOW;
    }
  }
  return FlashLights(LightColor::OFF, LightColor::TEAL, false);
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

bool Lights::is_red() const {
  return (light_color_ == LightColor::RED || light_color_ == LightColor::YELLOW ||
          light_color_ == LightColor::WHITE || light_color_ == LightColor::PINK);
}

}  // namespace lights

}  // namespace c2017
