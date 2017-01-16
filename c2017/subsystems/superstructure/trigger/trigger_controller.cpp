#include "trigger_controller.h"
#include <math.h>

namespace c2017 {

namespace trigger {

TriggerController::TriggerController() {
  auto ss_plant = muan::control::StateSpacePlant<1, 3, 1>(frc1678::trigger_controller::controller::A(), frc1678::trigger_controller::controller::B(), frc1678::trigger_controller::controller::C());
  
  // matrix math I don't understand and hope is correct
  controller_ = muan::control::StateSpaceController<1, 3, 1>(frc1678::trigger_controller::controller::K());
  controller_.u_min() = Eigen::Matrix<double, 1, 1>::Ones() * -12.0;
  controller_.u_max() = Eigen::Matrix<double, 1, 1>::Ones() * 12.0;
  observer_ = muan::control::StateSpaceObserver<1, 3, 1>(ss_plant, frc1678::trigger_controller::controller::L());
  
  // Tolerance in rad/sec
  velocity_tolerance_ = 5;
  at_goal_ = false;
}

TriggerOutputProto TriggerController::Update(const TriggerInputProto& input,
                                             const muan::wpilib::DriverStationProto& robot_state) {
  TriggerOutputProto output;
  TriggerStatusProto status;
  
  status->set_observed_velocity(observer_.x()[1]);
  status->set_goal_velocity(16 * muan::units::pi / 2);

  bool enable_outputs = !(robot_state->mode() == RobotMode::ESTOP ||
                          robot_state->mode() == RobotMode::DISABLED || robot_state->brownout());

  output->set_voltage(0);

  if (enable_outputs) {
    // Matrix stuff! Woo!
    Eigen::Matrix<double, 3, 1> r;
    r << 0.0, (muan::units::pi / 2 * goal_->balls_per_second()), 0.0;
    Eigen::Matrix<double, 1, 1> y;
    y << input->encoder_position();

    auto u = controller_.Update(observer_.x(), r);

    observer_.Update(u, y);
    output->set_voltage(u[0]);

    // Capping voltage
    if (output->voltage() < -12.) {
      output->set_voltage(-12.);
    } else if (output->voltage() > 12.) {
      output->set_voltage(12.);
    }
  }
  return output;
}

}  // trigger

}  // c2017
