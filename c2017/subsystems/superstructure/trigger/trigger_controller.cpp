#include "trigger_controller.h"
#include <math.h>
#include "c2017/queue_manager/queue_manager.h"

namespace c2017 {

namespace trigger {

//Constructor
TriggerController::TriggerController() :
      status_queue_(QueueManager::GetInstance().trigger_status_queue()) {
  auto ss_plant = muan::control::StateSpacePlant<1, 3, 1>(frc1678::trigger_controller::controller::A(),
                                                          frc1678::trigger_controller::controller::B(),
                                                          frc1678::trigger_controller::controller::C());

  // Matrix math
  controller_ = muan::control::StateSpaceController<1, 3, 1>(frc1678::trigger_controller::controller::K());
  controller_.u_min() = Eigen::Matrix<double, 1, 1>::Ones() * -12.0;
  controller_.u_max() = Eigen::Matrix<double, 1, 1>::Ones() * 12.0;
  observer_ =
      muan::control::StateSpaceObserver<1, 3, 1>(ss_plant, frc1678::trigger_controller::controller::L());

  // Tolerance in rad/sec
  velocity_tolerance_ = 1; //TODO tune this
}

TriggerOutputProto TriggerController::Update(const TriggerInputProto& input,
                                             const muan::wpilib::DriverStationProto& robot_state) {
  TriggerOutputProto output;
  TriggerStatusProto status;

  // Checking if E-Stop/brownout/disabled from driver station proto
  // Trigger should not be running if any of these are true
  bool enable_outputs = !(robot_state->mode() == RobotMode::ESTOP ||
                          robot_state->mode() == RobotMode::DISABLED ||
                          robot_state->brownout());

  if (enable_outputs && balls_per_second_ > 0) { // I didn't see the point in adding another if statement that does the same thing as this one
    // r is the goal
    Eigen::Matrix<double, 3, 1> r;
    r << 0.0, ((muan::units::pi / 2) * balls_per_second_), 0.0;
    // y is the input/sensor values
    Eigen::Matrix<double, 1, 1> y;
    y << input->encoder_position();

    // Cap voltage
    double voltage = output->voltage();
    muan::utils::Cap(voltage, -12, 12);
    output->set_voltage(voltage);

    // u is the motor value output
    auto u = controller_.Update(observer_.x(), r);
    
    observer_.Update(u, y);
    output->set_voltage(u[0]);
    output->set_voltage(0);
  } else {
    output->set_voltage(0);
  }

  status->set_observed_velocity(observer_.x()[1]);
  status->set_goal_velocity(balls_per_second_ * muan::units::pi / 2);
  status->set_position(observer_.x()[0]);
  
  status_queue_->WriteMessage(status);
  return output;
}

}  // trigger

}  // c2017
