#include "intake.h"

using namespace muan::control;
using namespace muan::units;

namespace frc1678{

namespace o2016 {

namespace intake {

Intake::Intake() {
  using namespace frc1678::intake_controller;
  auto ss_plant = StateSpacePlant<1, 2, 1> (controller::A(), controller::B(), controller::C());
  controller_ = StateSpaceController<1, 2, 1> (controller::K());
  controller_.u_min() = Eigen::Matrix<double, 1, 1>::Ones() * -12.0;
  controller_.u_max() = Eigen::Matrix<double, 1, 1>::Ones() * 12.0;
  observer_ = StateSpaceObserver<1, 2, 1> (ss_plant, controller::L());
  done = false;
}

Voltage Intake::Update(Angle goal, Angle sensor_value) {
  
}

Angle Intake::GetAngle() const {
  
}

void Intake::SetAngle(Angle theta) {
  //observer_.x()(0,0) = theta;
}

bool Intake::IsDone() const {
  return done;
}

}

}

}
