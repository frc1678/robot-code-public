#include "c2017/subsystems/lights/lights.h"

namespace c2017 {

namespace lights {
void  Lights::Update() {
  // auto x_status = QueueManager::x_status_queue()->MakeReader().ReadLastMessage();
  auto intake_group_goal_queue = QueueManager::GetInstance().intake_group_goal_queue().ReadLastMessage();
  auto vision_status_queue = QueueManager::GetInstance().vision_status_queue().ReadLastMessage():

  if (intake_group_goal_queue) {
    switch (intake_group_goal_queue.value().hp_load_type()) {
      case c2017::intake_group::HP_LOAD_NONE:
        light_color_ = LightColor::WHITE;
        break;
      case c2017::intake_group::HP_LOAD_BALLS:

        break;
      case c2017::intake_group::HP_LOAD_GEARS;
        
        break;
      case c2017::intake_group::HP_LOAD_BOTH;
        break;
    }
  }

  LightsOutputProto output;

  output->set_red(GetRed());
  
  QueueManager::GetInstance().lights_output_queue().WriteMessage(output);
}

bool Lights::GetRed() {
   return (light_color_ == LightColor::RED || light_color_ == LightColor::YELLOW || light_color_ == LightColor::WHITE || light_color_ == LightColor::PINK);
}

bool Lights::GetGreen() {
  return (light_color_ == LightColor::GREEN || light_color_ == LightColor::TEAL || light_color_ == Light_Color::YELLOW || light_color_ == WHITE)
}

bool Lights::GetBlue() {
  return (light_color_ == LightColor::BLUE || light_color_ == LightColor::TEAL || light_color_ == PINK || light_color_ == LightColor::WHITE)
}

}  //namespace lights

}  // namespace c2017
