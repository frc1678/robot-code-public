#include "catapult.h"
#include <iostream>

namespace o2016 {

namespace catapult {

Catapult::Catapult() :
  stop_(CatapultStop()),
  scoop_(Scoop()),
  catapult_countdown_(0),
  input_reader_(::o2016::QueueManager::GetInstance().catapult_input_queue().MakeReader()),
  goal_reader_(::o2016::QueueManager::GetInstance().catapult_goal_queue().MakeReader()) {}

void Catapult::Update() {

  CatapultOutputProto output;
  CatapultStatusProto status;
  CatapultInputProto input;
  CatapultGoalProto goal;

  input = *(input_reader_.ReadLastMessage());
  goal = *(goal_reader_.ReadLastMessage());

  // TODO(Lucas): Put calculations here
  status->set_scoop_goal(0 * rad);
  status->set_hardstop_goal(0 * rad);

  status->set_scoop_angle(scoop_.get_angle());
  status->set_scoop_angular_velocity(scoop_.get_angular_velocity());
  status->set_hardstop_angle(stop_.get_angle());
  status->set_hardstop_angular_velocity(stop_.get_angular_velocity());

  status->set_scoop_at_goal(scoop_.is_done());
  status->set_hardstop_at_goal(stop_.is_done());
  status->set_at_goal(status->hardstop_at_goal() && status->scoop_at_goal());

  status->set_disk_brake_locked(stop_.is_done() && goal->goal() == CatapultGoal::PREP_SHOT);
  status->set_can_shoot(status->at_goal() && status->disk_brake_locked() &&
                        (status->catapult_status() == RETRACTED ||
                        status->catapult_status() == EXTENDING));

  output->set_scoop_output(scoop_.Update(status->scoop_goal(), input->scoop_pot()));
  output->set_hardstop_output(stop_.Update(status->hardstop_goal(), input->hardstop_pot()));
  output->set_disc_brake_activate(status->hardstop_at_goal());

  if(goal->goal() == CatapultGoal::INTAKE || goal->goal() == CatapultGoal::PREP_SHOT) {
    catapult_countdown_--;
    output->set_catapult_extend(false);
    if(catapult_countdown_ < 0) {
      catapult_countdown_ = 0;
      status->set_catapult_status(RETRACTED);
    } else {
      status->set_catapult_status(RETRACTING);
    }
  } else if(goal->goal() == CatapultGoal::SHOOT && status->can_shoot()) {
    catapult_countdown_++;
    output->set_catapult_extend(true);
    if(catapult_countdown_ > extend_time) {
      catapult_countdown_ = extend_time;
      status->set_catapult_status(EXTENDED);
    } else {
      status->set_catapult_status(EXTENDING);
    }
  }

  ::o2016::QueueManager::GetInstance().catapult_output_queue().WriteMessage(output);
  ::o2016::QueueManager::GetInstance().catapult_status_queue().WriteMessage(status);
}

} // catapult

} // o2016
