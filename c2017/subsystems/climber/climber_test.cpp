#include "gtest/gtest.h"
#include "climber.h"
#include <iostream>

TEST(Climbing, HitsTheTop) {
    c2017::climber::ClimberGoalProto goal;
    c2017::climber::ClimberInputProto input;
    goal->set_climbing(true);
    input->set_current(150);
    input->set_position(5); // TODO find out what position is 
    c2017::climber::Climber test_climber;

    test_climber.SetGoal(goal);
    for(int i = 0; i < 1000; i++) {
        test_climber.Update(input);
    }

    c2017::climber::ClimberStatusProto test_status = test_climber.Status();
    c2017::climber::ClimberOutputProto test_output = test_climber.Output();


    EXPECT_TRUE(test_status->currently_climbing());
    EXPECT_TRUE(test_status->hit_top());
    EXPECT_EQ(test_output->voltage(), 0);
    }


TEST(Climbing, Climbing) {
    c2017::climber::ClimberGoalProto goal;
    c2017::climber::ClimberInputProto input;
    goal->set_climbing(true);
    input->set_current(50);
    input->set_position(5);
    c2017::climber::Climber test_climber;
    
    test_climber.SetGoal(goal);
    for (int i = 0; i < 1000; i++) {
      test_climber.Update(input);
    }

    c2017::climber::ClimberStatusProto test_status = test_climber.Status();
    c2017::climber::ClimberOutputProto test_output = test_climber.Output();


    EXPECT_TRUE(test_status->currently_climbing());
    EXPECT_FALSE(test_status->hit_top());
    EXPECT_EQ(test_output->voltage(), 12);
    }

TEST(Climbing, NotClimbing) {
    c2017::climber::ClimberGoalProto goal;
    c2017::climber::ClimberInputProto input;
    goal->set_climbing(false);
    input->set_current(0);
    input->set_position(5);
    c2017::climber::Climber test_climber;
    
    test_climber.SetGoal(goal);
    for (int i = 0; i < 1000; i++) {
      test_climber.Update(input);
    }

    c2017::climber::ClimberStatusProto test_status = test_climber.Status();
    c2017::climber::ClimberOutputProto test_output = test_climber.Output();


    EXPECT_FALSE(test_status->currently_climbing());
    EXPECT_FALSE(test_status->hit_top());
    EXPECT_EQ(test_output->voltage(), 0);
    }
