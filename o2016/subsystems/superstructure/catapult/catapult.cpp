#include "catapult.h"
#include <iostream>

namespace o2016 {

namespace catapult {

Catapult::Catapult() :
  stop_(CatapultStop()),
  scoop_(Scoop()),
  catapult_countdown_(0) {}

void Catapult::Update(CatapultInput input, CatapultGoal goal) {
  // TODO(Lucas): Put calculations here
  status_.set_scoop_goal(0 * rad);
  status_.set_hardstop_goal(0 * rad);

  status_.set_scoop_angle(scoop_.get_angle());
  status_.set_scoop_angular_velocity(scoop_.get_angular_velocity());
  status_.set_hardstop_angle(stop_.get_angle());
  status_.set_hardstop_angular_velocity(stop_.get_angular_velocity());

  status_.set_scoop_at_goal(scoop_.is_done());
  status_.set_hardstop_at_goal(stop_.is_done());
  status_.set_at_goal(status_.hardstop_at_goal() && status_.scoop_at_goal());

  status_.set_disk_brake_locked(stop_.is_done() && goal.goal() == CatapultGoal::PREP_SHOT);
  status_.set_can_shoot(status_.at_goal() && status_.disk_brake_locked() &&
                        (status_.catapult_status() == RETRACTED ||
                        status_.catapult_status() == EXTENDING));

  output_.set_scoop_output(scoop_.Update(status_.scoop_goal(), input.scoop_pot()));
  output_.set_hardstop_output(stop_.Update(status_.hardstop_goal(), input.hardstop_pot()));
  output_.set_disc_brake_activate(status_.hardstop_at_goal());

  if(goal.goal() == CatapultGoal::INTAKE || goal.goal() == CatapultGoal::PREP_SHOT) {
    catapult_countdown_--;
    output_.set_catapult_extend(false);
    if(catapult_countdown_ < 0) {
      catapult_countdown_ = 0;
      status_.set_catapult_status(RETRACTED);
    } else {
      status_.set_catapult_status(RETRACTING);
    }
  } else if(goal.goal() == CatapultGoal::SHOOT && status_.can_shoot()) {
    catapult_countdown_++;
    output_.set_catapult_extend(true);
    if(catapult_countdown_ > extend_time) {
      catapult_countdown_ = extend_time;
      status_.set_catapult_status(EXTENDED);
    } else {
      status_.set_catapult_status(EXTENDING);
    }
  }
}

CatapultOutput Catapult::get_output() const { return output_; }

CatapultStatus Catapult::get_status() const { return status_; }

} // catapult

} // o2016
