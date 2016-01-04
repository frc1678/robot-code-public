#include <fstream>
#include <tuple>
#include "gtest/gtest.h"
#include "unitscpp/unitscpp.h"
#include "../control/trapezoidal_motion_profile.h"

TEST(DriveTest, DistDriven) {
  Length distance = 5*ft;
  TrapezoidalMotionProfile<Length> motion_profile(distance, maxHighRobotSpeed, maxHighRobotSpeed / (2*s));
  Time t;
  Velocity last_speed = motion_profile.calculate_speed(0*s);
  for (t = 0; !motion_profile.finished(t); t+=.05*s) {
    Velocity cur_speed = motion_profile.calculate_speed(t);
    Acceleration accel = (cur_speed - last_speed) / (.05*s);
    EXPECT_LE(cur_speed, maxHighRobotSpeed);
    EXPECT_LE(accel.to(ft/s/s), (maxHighRobotSpeed / (2*s)).to(ft/s/s) + .0001) << "Robot accelerating too quickly";
    EXPECT_GE(accel.to(ft/s/s), (-maxHighRobotSpeed / (2*s)).to(ft/s/s) - .0001) << "Robot deccelerating too quickly";
    last_speed = cur_speed;
  }
  EXPECT_NEAR(motion_profile.calculate_distance(t).to(ft), distance.to(ft), 0.00001);
}
