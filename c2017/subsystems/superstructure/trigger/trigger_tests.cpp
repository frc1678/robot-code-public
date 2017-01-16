#include "gtest/gtest.h"
#include "c2017/subsystems/superstructure/trigger/trigger_constants.h"
#include "c2017/subsystems/superstructure/trigger/trigger_controller.h"
#include "muan/control/state_space_plant.h"
#include "muan/control/state_space_controller.h"
#include <iostream>
#include <fstream>
#include <math.h>


TEST(TriggerController, ZeroInput) {
  c2017::trigger::TriggerInputProto input;
  c2017::trigger::TriggerOutputProto output;
  c2017::trigger::TriggerStatusProto status;
  c2017::trigger::TriggerGoalProto goal;

  c2017::trigger::TriggerController trigger_;
  auto plant = muan::control::StateSpacePlant<1, 3, 1>(frc1678::trigger_controller::controller::A(), 
                                                       frc1678::trigger_controller::controller::B(), 
                                                       frc1678::trigger_controller::controller::C());

  plant.x()[0] = 0.0;
  plant.x()[1] = 0.0;
  plant.x()[2] = 0.0;

  for (int i = 0; i <= 1000; i++) {
    goal->set_balls_per_second(0);
    input->set_encoder_position(plant.x()[0]);
     
    trigger_.SetGoal(goal);

    output = trigger_.Update(input);

    plant.Update((Eigen::Matrix<double, 1, 1>() << output->voltage()).finished()); 
  
    //Making sure voltage is capped
    EXPECT_NEAR(output->voltage(), 0., 12.);
  }
  //We need to translate rotations per second back into balls per second.
  //If one trigger rotates 2 times per second, it is shooting 4 balls per second.
  //If two triggers rotate 2 times per second, they are collectively shooting 8 balls per second.
  //
  //x.()[0] = change in position since the thingy started moving
  //x.()[1] = velocity
  //x.()[2] = displacement??
  
  //Test that the bps is what we want
  //plant.x()[1] * (2 * pi) changes velocity into revolutions per second
  //(...) * 4 -> multiplying by 2 for balls per rotation and by 2 again because there are two triggers
  EXPECT_NEAR(goal->balls_per_second(), (plant.x()[1] * (2 * muan::units::pi)) * 4, 1e-3); 
  //Testing that the velocity is close to what we want, based on velocity tolerance
  EXPECT_NEAR(plant.x()[1], 0, trigger_.get_velocity_tolerance());
  /*TODO
   * expect that we're at the goal
   * 
   * test 0 (done)
   * test 75bps
   * test no output/brownout/disabled
   */
}


TEST(TriggerController, NormalInput) {
  c2017::trigger::TriggerInputProto input;
  c2017::trigger::TriggerOutputProto output;
  c2017::trigger::TriggerStatusProto status;
  c2017::trigger::TriggerGoalProto goal;

  c2017::trigger::TriggerController trigger_;
  auto plant = muan::control::StateSpacePlant<1, 3, 1>(frc1678::trigger_controller::controller::A(), 
                                                       frc1678::trigger_controller::controller::B(), 
                                                       frc1678::trigger_controller::controller::C());

  plant.x()[0] = 0.0;
  plant.x()[1] = 0.0;
  plant.x()[2] = 0.0;

  for (int i = 0; i <= 1000; i++) {
    goal->set_balls_per_second(16);
    input->set_encoder_position(plant.x()[0]);
     
    trigger_.SetGoal(goal);

    output = trigger_.Update(input);

    plant.Update((Eigen::Matrix<double, 1, 1>() << output->voltage()).finished()); 
  
    //Making sure voltage is capped
    EXPECT_NEAR(output->voltage(), 0., 12.);
  }
  //We need to translate rotations per second back into balls per second.
  //If one trigger rotates 2 times per second, it is shooting 4 balls per second.
  //If two triggers rotate 2 times per second, they are collectively shooting 8 balls per second.
  //x.()[0] = change in position since the thingy started moving
  //x.()[1] = velocity
  //x.()[2] = displacement??
  
  //(...) * 4 -> multiplying by 2 for balls per rotation and by 2 again because there are two triggers
  EXPECT_NEAR(goal->balls_per_second() * muan::units::pi / 2, plant.x()[1], 1); 
  //Testing that the velocity is close to what we want, based on velocity tolerance
  EXPECT_NEAR(plant.x()[1], (4 * (muan::units::pi / 2)), trigger_.get_velocity_tolerance());
}
