#include "catapult.h"
#include <iostream>

Catapult::Catapult() :
  stop_(CatapultStop()),
  scoop_(Scoop()),
  cylinder_extended_(false) {}

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

  status_.set_cylinder_extended(cylinder_extended_);

  // TODO(Lucas): Make it calibrate
  status_.set_calibrated(true); //stop.is_calibrated();
  // TODO(Lucas): Put this on a timer
  status_.set_disk_brake_locked(stop_.is_done());
  status_.set_can_shoot(status_.scoop_terminated() && status_.stop_terminated() &&
                     status_.disk_brake_locked() && status_.calibrated() &&
                     !status_.cylinder_extended());
                     
  output_.set_scoop_output(scoop_.Update(status_.scoop_goal(), input.scoop_pot()));
  // TODO(Lucas): Make it calibrate
  output_.set_stop_output(stop_.Update(status_.stop_goal(), input.stop_encoder())); // , stop_pot
  output_.set_disc_brake_activate(status_.stop_terminated());
  // Tucking takes precedence over shooting
  if(goal.goal() == CatapultGoal::TUCK) {
    // TODO(Lucas): Put this on a timer
    cylinder_extended_ = false;
  } else if(goal.goal() == CatapultGoal::SHOOT && status_.can_shoot()) {
    // TODO(Lucas): Put this on a timer
    cylinder_extended_ = true;
  }
}

CatapultOutput Catapult::get_output() const { return output_; }

CatapultStatus Catapult::get_status() const { return status_; }
