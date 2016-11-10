#include "gtest/gtest.h"
#include "intake.h"
#include "o2016/subsystems/superstructure/intake/intake_constants.h"
#include "muan/control/state_space_plant.h"
#include "muan/control/state_space_controller.h"
#include <iostream>
#include <fstream>

using namespace muan::control;
using namespace frc1678::intake_controller;
using namespace o2016::intake;


TEST(IntakeController, Universal) {
  IntakeInputProto input;
  IntakeOutputProto output;
  IntakeStatusProto status;
  IntakeGoalProto goal;

  bool index_click;

  Intake intake_;
  auto plant = StateSpacePlant<1, 3, 1>(controller::A(), controller::B(), controller::C());

  plant.x(0) = 0.5;
  plant.x(1) = 0.0;
  plant.x(2) = 0.0;

  
  for(int i = 0; i <= 100000; i++) {
    // This is hacky
    index_click = plant.x(0) > 0.3 && plant.x(0) < 0.31;

    input->set_encoder_position(plant.x(0) + 10.);
    input->set_index_click(index_click);

    goal->set_goal_angle(1.0);
    goal->set_intake_speed(RollerGoal::FORWARD);

    output = intake_.Update(input, goal);
    status = intake_.Status();

    plant.Update((Eigen::Matrix<double, 1, 1>() << output->arm_voltage()).finished());


    EXPECT_NEAR(output->arm_voltage(), 0., 12.);
    EXPECT_NEAR(output->roller_voltage(), 0, 12.);
  }


  EXPECT_TRUE(status->is_calibrated());
  EXPECT_TRUE(status->at_goal());
  EXPECT_NEAR(status->intake_position(), 1.0, 0.05);
  EXPECT_EQ(status->current_roller_goal(), RollerGoal::FORWARD);
}

