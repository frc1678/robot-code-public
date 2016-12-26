#include "gtest/gtest.h"
#include "drivetrain.h"

using test1::Drivetrain_Output;
using test1::Drivetrain;

//Full forward
TEST(DriveTrainTest, FullForward) {
    Drivetrain drive;
    Drivetrain_Output output = drive.Update(1, 0, false);
    EXPECT_EQ(output.left_voltage, 1);
    EXPECT_EQ(output.right_voltage, 1);
}

//no movement
TEST(DriveTrainTest, NoMove) {
    Drivetrain drive;
    Drivetrain_Output output = drive.Update(0, 0, false);
    EXPECT_EQ(output.left_voltage, 0);
    EXPECT_EQ(output.right_voltage, 0);
}

TEST(DriveTrainTest, NoMoveRight) {
    Drivetrain drive;
    Drivetrain_Output output = drive.Update(0, 1, false);
    EXPECT_EQ(output.left_voltage, 0);
    EXPECT_EQ(output.right_voltage, 0);
}

TEST(DriveTrainTest, NoMoveLeft) {
    Drivetrain drive;
    Drivetrain_Output output = drive.Update(0, -1, false);
    EXPECT_EQ(output.left_voltage, 0);
    EXPECT_EQ(output.right_voltage, 0);
}

//Full Turn Right
TEST(DriveTrainTest, TurnRight) {
    Drivetrain drive;
    Drivetrain_Output output = drive.Update(1, 1, false);
    EXPECT_EQ(output.left_voltage, 1);
    EXPECT_EQ(output.right_voltage, 0);
}

//Full Turn Left
TEST(DriveTrainTest, TurnLeft) {
    Drivetrain drive;
    Drivetrain_Output output = drive.Update(1, -1, false);
    EXPECT_EQ(output.left_voltage, 0);
    EXPECT_EQ(output.right_voltage, 1);
}

//Quickturn Mode turn left
TEST(DriveTrainTest, QTTurnLeft) {
    Drivetrain drive;
    Drivetrain_Output output = drive.Update(1, -1, true);
    EXPECT_EQ(output.left_voltage, 1);
    EXPECT_EQ(output.right_voltage, -1);
}

//Quickturn Mode turn right
TEST(DriveTrainTest, QTTurnRight) {
    Drivetrain drive;
    Drivetrain_Output output = drive.Update(1, 1, true);
    EXPECT_EQ(output.left_voltage, -1);
    EXPECT_EQ(output.right_voltage, 1);
}

//Full Backward
TEST(DriveTrainTest, FullBackward) {
    Drivetrain drive;
    Drivetrain_Output output = drive.Update(-1, 0, false);
    EXPECT_EQ(output.left_voltage, -1);
    EXPECT_EQ(output.right_voltage, -1);
}

