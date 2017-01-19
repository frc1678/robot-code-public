#include "gtest/gtest.h"
#include "climber.h"
#include <iostream>

TEST(Climbing, ClimbsToTheTop) {
  c2017::climber::ClimberGoalProto goal;
  c2017::climber::ClimberInputProto input;
  c2017::climber::ClimberOutputProto output;
  muan::wpilib::DriverStationProto ds_status;
  ds_status->set_mode(RobotMode::TELEOP);
  goal->set_climbing(true);
  input->set_position(100);  // position helps find the rate of the encoder. Encoder has been running
                             // throughout the match as the shooter so its big.
  c2017::climber::Climber test_climber;

  test_climber.SetGoal(goal);
  for (double t = 0.005; t < 2; t += 0.005) {
    double d;
    if (t < 1) {
      d = t;
    } else {
      d = 1;
    }
    input->set_position(d);
    output = test_climber.Update(input, ds_status);
    if (d < 1) {
      EXPECT_NEAR(output->voltage(), 12, 1e-5);
    }
  }

  c2017::climber::ClimberStatusProto test_status = test_climber.Status();

  EXPECT_TRUE(test_status->currently_climbing());
  EXPECT_TRUE(test_status->hit_top());
  EXPECT_NEAR(output->voltage(), 0, 1e-5);
}

TEST(Climbing, Disabled) {
  c2017::climber::ClimberGoalProto goal;
  c2017::climber::ClimberInputProto input;
  c2017::climber::ClimberOutputProto output;
  muan::wpilib::DriverStationProto ds_status;
  ds_status->set_mode(RobotMode::DISABLED);
  goal->set_climbing(true);
  input->set_position(0);  // position helps find the rate of the encoder.
  c2017::climber::Climber test_climber;

  test_climber.SetGoal(goal);
  for (double t = 0; t < 2; t += 0.005) {
    output = test_climber.Update(input, ds_status);
    double d;
    if (t < 1) {
      d = t;
    } else {
      d = 1;
    }
    input->set_position(d);
    c2017::climber::ClimberStatusProto test_status = test_climber.Status();

    EXPECT_FALSE(test_status->currently_climbing());
    EXPECT_FALSE(test_status->hit_top());
    EXPECT_NEAR(output->voltage(), 0, 1e-5);
  }
}
