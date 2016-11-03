#include "intake_controller.h"

using namespace muan::control;
using namespace muan::units;

namespace frc1678{

namespace o2016 {

namespace intake {

IntakeController::IntakeController() {
  using namespace frc1678::intake_controller;

  auto ss_plant = StateSpacePlant<1, 2, 1> (controller::A(), controller::B(), controller::C());
  controller_ = StateSpaceController<1, 2, 1> (controller::K());
  controller_.u_min() = Eigen::Matrix<double, 1, 1>::Ones() * -12.0;
  controller_.u_max() = Eigen::Matrix<double, 1, 1>::Ones() * 12.0;
  observer_ = StateSpaceObserver<1, 2, 1> (ss_plant, controller::L());
  done = false;
  
  angle_tolerance_ = 1 * deg;
  velocity_tolerance_ = 5 * deg / s;

}

Voltage IntakeController::Update(sensor_value) {
  auto y = (Eigen::Matrix<double, 1, 1>() << 



}

Angle IntakeController::GetAngle() const {
  return observer_.x()(0, 0);
}

void IntakeController::SetAngle(Angle theta) {
  observer_.x()(0, 0) = theta;
}

bool IntakeController::AtGoal() const {
  return at_goal_;
}

}

}

}
