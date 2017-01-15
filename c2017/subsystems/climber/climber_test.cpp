#include "gtest/gtest.h"
#include "climber.h"
#include <iostream>

TEST(Climbing, HitsTheTop) {
    ClimberGoalProto goal;
    goal.climbing = true;
    ClimberInputProto input;
    input.current = 150;
    input.position = ; // TODO find out what position is 
    EXPECT_TRUE(is_climbing_)
    EXPECT_TRUE(hit_top_)
    EXPECT_EQ(voltage_, 0)
    }
