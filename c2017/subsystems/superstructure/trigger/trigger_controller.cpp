#include "trigger_controller.h"
#include <math.h>

using namespace muan::control;
using namespace muan::units;

namespace c2017 {

namespace trigger {

using namespace ::muan::control;
using namespace ::frc1678::trigger_controller;

TriggerController::TriggerController() {
  auto ss_plant = StateSpacePlant<1, 3, 1>(controller::A(), controller::B(),
                                           controller::C());
    //matrix math I don't understand and hope is correct
  controller_ = StateSpaceController<1, 3, 1>(controller::K());
  controller_.u_min() = Eigen::Matrix<double, 1, 1>::Ones() * -12.0;
  controller_.u_max() = Eigen::Matrix<double, 1, 1>::Ones() * 12.0;
  observer_ = StateSpaceObserver<1, 3, 1>(ss_plant, controller::L());
//tlerance in rad/sec
  velocity_tolerance_ = 5;
  at_goal_ = false;
}

TriggerOutputProto TriggerController::Update(TriggerInputProto input) {
  TriggerOutputProto output;
  //The current aim is 16 bps with two triggers.
  //That means 8 bps in each trigger.
  //Each trigger pushs through 2 balls per rotation,
  //Which means the trigger needs to rotate 4 times per second.

//more matrix math I don't understand - ask Kyle
  Eigen::Matrix<double, 3, 1> r;
  r << 0.0, (8.0 * pi), 0.0;

  Eigen::Matrix<double, 1, 1> y;
  y << input->encoder_position();

  auto u = controller_.Update(observer_.x(), r);

  observer_.Update(u, y);

  output->set_voltage(u[0]);

  return output; //outputs velocity
}



} //trigger

} //c2017
