#include “gtest/gtest.h”
#include “drivetrain.h”

TEST(DriveTrainTest, FullForwards) {
  DriveTrain bob;
  tuple = bob.Update(1, 1, true);
  EXPECT_EQ(std::get<0>(tuple), 12);
  EXPECT_EQ(std::get<1>(tuple), -12);
}

TEST(DriveTrainTest, FullBackwards) {
  DriveTrain bob;
  tuple = bob.Update(
}
