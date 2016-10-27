#include "catapult.h"
#include <iostream>

Catapult::Catapult(std::shared_ptr<muan::queues::MessageQueue<CatapultInput, 10>> input_queue,
                   std::shared_ptr<muan::queues::MessageQueue<CatapultGoal, 10>> goal_queue,
                   std::shared_ptr<muan::queues::MessageQueue<CatapultOutput, 10>> output_queue,
                   std::shared_ptr<muan::queues::MessageQueue<CatapultStatus, 10>> status_queue) :
  stop_(CatapultStop()),
  scoop_(Scoop()),
  cylinder_extended_(false),
  input_(input_queue->MakeReader()),
  goal_(goal_queue->MakeReader()),
  output_(output_queue),
  status_(status_queue) {
    std::cout << "got here" << std::endl; }

void Catapult::Update() {
  CatapultInput input;
  CatapultGoal goal;
  CatapultOutput output;
  CatapultStatus status;

  while(auto val = input_.ReadMessage()) {
    input = *val;
  }
  while(auto val = goal_.ReadMessage()) {
    goal = *val;
  }

  // TODO(Lucas): Put calculations here
  status.scoop_goal = 0 * rad;
  status.stop_goal = 0 * rad;

  status.scoop_angle = scoop_.get_angle();
  status.scoop_angular_velocity = scoop_.get_angular_velocity();
  status.stop_angle = stop_.get_angle();
  status.stop_angular_velocity = stop_.get_angular_velocity();

  status.scoop_terminated = scoop_.is_done();
  status.stop_terminated = stop_.is_done();

  status.cylinder_extended = cylinder_extended_;

  // TODO(Lucas): Make it calibrate
  status.calibrated = true; //stop.is_calibrated();
  // TODO(Lucas): Put this on a timer
  status.disk_brake_locked = stop_.is_done();
  status.can_shoot = status.scoop_terminated && status.stop_terminated &&
                     status.disk_brake_locked && status.calibrated &&
                     !status.cylinder_extended;
                     
  output.scoop_output = scoop_.Update(status.scoop_goal, input.scoop_pot);
  // TODO(Lucas): Make it calibrate
  output.stop_output = stop_.Update(status.stop_goal, input.stop_encoder); // , stop_pot
  output.disc_brake_activate = status.stop_terminated;
  // Tucking takes precedence over shooting
  if(goal.should_tuck) {
    // TODO(Lucas): Put this on a timer
    cylinder_extended_ = false;
  } else if(goal.should_shoot && status.can_shoot) {
    // TODO(Lucas): Put this on a timer
    cylinder_extended_ = true;
  }

  output_->WriteMessage(output);
  status_->WriteMessage(status);
}
