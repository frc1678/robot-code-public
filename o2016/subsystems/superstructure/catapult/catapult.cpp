#include "catapult.h"
#include <iostream>

Catapult::Catapult() :
  stop_(CatapultStop()),
  scoop_(Scoop()),
  cylinder_countdown_(0) {}

void Catapult::Update(CatapultInput input, CatapultGoal goal) {
  // TODO(Lucas): Put calculations here
  status_.set_scoop_goal(0 * rad);
  status_.set_stop_goal(0 * rad);

  status_.set_scoop_angle(scoop_.get_angle());
  status_.set_scoop_angular_velocity(scoop_.get_angular_velocity());
  status_.set_stop_angle(stop_.get_angle());
  status_.set_stop_angular_velocity(stop_.get_angular_velocity());

  status_.set_scoop_terminated(scoop_.is_done());
  status_.set_stop_terminated(stop_.is_done());
  status_.set_terminated(status_.stop_terminated() && status_.scoop_terminated());

  if(cylinder_countdown == 0) {
  } else if(cylinder_countdown == extend_time) { 
    status_.set_cylinder_status(CatapultStatus::EXTENDED);
  } else { 
    status_.set_cylinder_status(CatapultStatus::WAITING);
  }

  status_.set_disk_brake_locked(stop_.is_done() && goal.goal() == CatapultStatus::PREP_SHOT);
  status_.set_can_shoot(status_.terminated() && status_.disk_brake_locked() &&
                        (status_.cylinder_status() == CatapultStatus::RETRACTED ||
                        status_.cylinder_status() == CatapultStatus::EXTENDING));
                     
  output_.set_scoop_output(scoop_.Update(status_.scoop_goal(), input.scoop_pot()));
  output_.set_stop_output(stop_.Update(status_.stop_goal(), input.stop_pot()));
  output_.set_disc_brake_activate(status_.stop_terminated());

  if(goal.goal() == CatapultGoal::INTAKE || goal.goal() == CatapultGoal::PREP_SHOT) {
    cylinder_countdown_--;
    output_.set_cylinder_extend(false);
    if(cylinder_countdown < 0) {
      cylinder_countdown = 0;
      status_.set_cylinder_status(CatapultStatus::RETRACTED);
    } else { 
      status_.set_cylinder_status(CatapultStatus::RETRACTING);
    }
  } else if(goal.goal() == CatapultGoal::SHOOT && status_.can_shoot()) {
    cylinder_countdown_++;
    output_.set_cylinder_extend(true);
    if(cylinder_countdown > extend_time) { 
      cylinder_countdown = extend_time;
      status_.set_cylinder_status(CatapultStatus::EXTENDED);
    } else { 
      status_.set_cylinder_status(CatapultStatus::EXTENDING);
    }
  }
}

CatapultOutput Catapult::get_output() const { return output_; }

CatapultStatus Catapult::get_status() const { return status_; }
