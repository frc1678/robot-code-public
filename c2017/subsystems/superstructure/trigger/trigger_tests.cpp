#include "gtest/gtest.h"
#include "c2017/subsystems/superstructure/trigger/trigger_constants.h"
#include "c2017/subsystems/superstructure/trigger/trigger_controller.h"
#include "muan/control/state_space_plant.h"
#include "muan/control/state_space_controller.h"
#include <iostream>
#include <fstream>

using namespace muan::control;
using namespace frc1678::trigger_controller;
using namespace c2017::trigger;

TEST(TriggerController, ZeroInput) {
  TriggerInputProto input;
  TriggerOutputProto output;
  TriggerStatusProto status;
  TriggerGoalProto goal;

  TriggerController trigger_;
  auto plant = StateSpacePlant<1, 3, 1>(controller::A(), controller::B(), controller::C());

  plant.x(0) = 0.0;
  plant.x(1) = 0.0;
  plant.x(2) = 0.0;

  for (int i = 0; i <= 1000; i++) {
    goal->set_balls_per_seconda(0);
    input->set_encoder_position(plant.x(0) + 10.);
   
    output = trigger_.Update(input);

    plant.Update((Eigen::Matrix<double, 1, 1>() << output->voltage()).finished()); 
   
    EXPECT_NEAR(output->voltage(), 0., 12.)
  }
  EXPECT_NEAR(plant.x()[1], 0, trigger_.velocity_tolerance)
  /*TODO
   * expect that we're at the goal
   * test 0 (done)
   * test 75bps
   * test no output/brownout/disabled
   */
}
