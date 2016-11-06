#include "catapult.h"
#include <iostream>

namespace o2016 {

namespace catapult {

Catapult::Catapult() :
  stop_(CatapultStop()),
  scoop_(Scoop()),
  catapult_countdown_(0),
  catapult_status_(RETRACTED),
  // Intaking is the default state to avoid collisions
  state_(CatapultStatus::INTAKING) {}

void Catapult::Update(CatapultInputProto input, CatapultGoalProto goal, Length dist_to_target) {

  Voltage scoop_output = scoop_.Update(status_->scoop_goal(), input->scoop_pot());
  Voltage hardstop_output = stop_.Update(status_->hardstop_goal(), input->hardstop_pot());

  status_->set_scoop_angle(scoop_.get_angle());
  status_->set_scoop_angular_velocity(scoop_.get_angular_velocity());
  status_->set_hardstop_angle(stop_.get_angle());
  status_->set_hardstop_angular_velocity(stop_.get_angular_velocity());

  status_->set_scoop_at_goal(scoop_.is_done());
  status_->set_hardstop_at_goal(stop_.is_done());
  status_->set_at_goal(status_->hardstop_at_goal() && status_->scoop_at_goal());

  if(state_ == CatapultStatus::SHOOTING) { 
    // We just want it to stay where it is when shooting
    status_->set_scoop_goal(status_->scoop_angle());
    status_->set_hardstop_goal(status_->hardstop_angle());

    output_->set_disc_brake_activate(true);
    status_->set_can_shoot(true);

    output_->set_scoop_output(0 * V);
    output_->set_hardstop_output(0 * V);
    output_->set_catapult_extend(true);

    // After firing, go to preping_shot
    if(catapult_status_ == EXTENDED) {
      state_ = CatapultStatus::PREPING_SHOT;
    }

  } else if(state_ == CatapultStatus::PREPING_SHOT) { 
    // TODO: Put calculations here
    status_->set_scoop_goal(1 * rad);
    status_->set_hardstop_goal(0 * rad);

    output_->set_disc_brake_activate(false);
    // Don't shoot if control loops aren't done or if catapult isn't down
    status_->set_can_shoot(status_->at_goal() && catapult_status_ == RETRACTED);

    output_->set_scoop_output(scoop_output);
    output_->set_hardstop_output(hardstop_output);
    output_->set_catapult_extend(false);

    if(goal->goal() == CatapultGoal::SHOOT && status_->can_shoot()) {
      state_ = CatapultStatus::SHOOTING;
    }
    // Can't be in intake state if catapult is up, that could cause collisions
    if(goal->goal() == CatapultGoal::INTAKE && catapult_status_ == RETRACTED) {
      state_ = CatapultStatus::INTAKING;
    }

  } else if(state_ == CatapultStatus::INTAKING) { 
    status_->set_scoop_goal(0 * rad);
    status_->set_hardstop_goal(status_->hardstop_angle());

    output_->set_disc_brake_activate(true);
    status_->set_can_shoot(false);

    output_->set_scoop_output(scoop_output);
    output_->set_hardstop_output(hardstop_output);
    output_->set_catapult_extend(false);

    // Not possible to fire without aiming first
    if(goal->goal() != CatapultGoal::INTAKE) {
      state_ = CatapultStatus::PREPING_SHOT;
    }
  }

  // Time to lock is less than time to fire, so it can be assumed to be 0
  status_->set_disc_brake_locked(output_->disc_brake_activate());

  // Count up when extending, down when retracting.
  // If an endpoint is reached the cylinder has stopped moving.
  if(output_->catapult_extend()) {
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

  status_->set_state(state_);

  status_->set_catapult_status(catapult_status_);

  std::cout<<status_->scoop_angle()<<" "<<status_->scoop_angular_velocity()<<std::endl;
}

CatapultOutputProto Catapult::output() { return output_; }

CatapultStatusProto Catapult::status() { return status_; }

} // catapult

} // o2016
