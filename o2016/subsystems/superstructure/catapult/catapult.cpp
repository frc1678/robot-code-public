#include "catapult.h"
#include <iostream>

namespace o2016 {

namespace catapult {

Catapult::Catapult() :
  stop_(CatapultStop()),
  scoop_(Scoop()),
  catapult_countdown_(0),
  catapult_status_(RETRACTED),
  state_(CatapultStatus::INTAKING),
  input_reader_(::o2016::QueueManager::GetInstance().catapult_input_queue().MakeReader()),
  goal_reader_(::o2016::QueueManager::GetInstance().catapult_goal_queue().MakeReader()) {}

void Catapult::Update() {

  CatapultOutputProto output;
  CatapultStatusProto status;
  CatapultInputProto input;
  CatapultGoalProto goal;

  input = *(input_reader_.ReadLastMessage());
  goal = *(goal_reader_.ReadLastMessage());

  Voltage scoop_output = scoop_.Update(status->scoop_goal(), input->scoop_pot());
  Voltage hardstop_output = stop_.Update(status->hardstop_goal(), input->hardstop_pot());

  status->set_scoop_angle(scoop_.get_angle());
  status->set_scoop_angular_velocity(scoop_.get_angular_velocity());
  status->set_hardstop_angle(stop_.get_angle());
  status->set_hardstop_angular_velocity(stop_.get_angular_velocity());

  status->set_scoop_at_goal(scoop_.is_done());
  status->set_hardstop_at_goal(stop_.is_done());
  status->set_at_goal(status->hardstop_at_goal() && status->scoop_at_goal());

  if(state_ == CatapultStatus::SHOOTING) { 
    status->set_scoop_goal(status->scoop_angle());
    status->set_hardstop_goal(status->hardstop_angle());

    output->set_disc_brake_activate(true);
    status->set_can_shoot(true);

    output->set_scoop_output(0 * V);
    output->set_hardstop_output(0 * V);
    output->set_catapult_extend(true);

    if(catapult_status_ == EXTENDED) {
      state_ = CatapultStatus::PREPING_SHOT;
    }

  } else if(state_ == CatapultStatus::PREPING_SHOT) { 
    // TODO(Lucas): Put calculations here
    status->set_scoop_goal(1 * rad);
    status->set_hardstop_goal(0 * rad);

    output->set_disc_brake_activate(false);
    status->set_can_shoot(status->at_goal() && catapult_status_ == RETRACTED);

    output->set_scoop_output(scoop_output);
    output->set_hardstop_output(hardstop_output);
    output->set_catapult_extend(false);

    if(goal->goal() == CatapultGoal::SHOOT && status->can_shoot()) {
      state_ = CatapultStatus::SHOOTING;
    }
    if(goal->goal() == CatapultGoal::INTAKE && catapult_status_ == RETRACTED) {
      state_ = CatapultStatus::INTAKING;
    }

  } else if(state_ == CatapultStatus::INTAKING) { 
    status->set_scoop_goal(0 * rad);
    status->set_hardstop_goal(status->hardstop_angle());

    output->set_disc_brake_activate(true);
    status->set_can_shoot(false);

    output->set_scoop_output(scoop_output);
    output->set_hardstop_output(hardstop_output);
    output->set_catapult_extend(false);

    if(goal->goal() != CatapultGoal::INTAKE) {
      state_ = CatapultStatus::PREPING_SHOT;
    }
  }

  // Time to lock is less than time to fire, so it can be assumed to be 0
  status->set_disc_brake_locked(output->disc_brake_activate());

  if(output->catapult_extend()) {
    catapult_countdown_++;
    if(catapult_countdown_ > extend_time) {
      catapult_countdown_ = extend_time;
      catapult_status_ = EXTENDED;
    } else {
      catapult_status_ = EXTENDING;
    }
  } else {
    catapult_countdown_--;
    if(catapult_countdown_ < 0) {
      catapult_countdown_ = 0;
      catapult_status_ = RETRACTED;
    } else {
      catapult_status_ = RETRACTING;
    }
  }

  status->set_state(state_);

  status->set_catapult_status(catapult_status_);

  ::o2016::QueueManager::GetInstance().catapult_output_queue().WriteMessage(output);
  ::o2016::QueueManager::GetInstance().catapult_status_queue().WriteMessage(status);
}

} // catapult

} // o2016
