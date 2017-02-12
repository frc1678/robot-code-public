#include "c2017/subsystems/superstructure/ground_gear_intake/ground_gear_intake.h"
#include "c2017/subsystems/superstructure/ground_gear_intake/queue_types.h"
#include "gtest/gtest.h"

class GroundGearIntakeTest : public ::testing::Test {
 public:
  void Update(double current, bool robot_disabled) {
    for (int i = 0; i < 1000; i++) {
      input->set_current(current);
      if (robot_disabled) {
        robot_state.set_mode(RobotMode::DISABLED);  //  for testing purposes
      } else {
        robot_state.set_mode(RobotMode::TELEOP);
      }
      robot_state.set_brownout(false);
      output = gear_intake.Update(input, robot_state);
    }
  }

  void SetGoal(c2017::ground_gear_intake::Goal goal_state) {
    goal->set_goal(goal_state);
    gear_intake.SetGoal(goal);
  }

  double GetVoltage() { return output->roller_voltage(); }
  bool GetIntakeState() { return output->intake_down(); }

 private:
  c2017::ground_gear_intake::GroundGearIntake gear_intake;
  c2017::ground_gear_intake::GroundGearIntakeInputProto input;
  c2017::ground_gear_intake::GroundGearIntakeGoalProto goal;
  DriverStationStatus robot_state;
  c2017::ground_gear_intake::GroundGearIntakeOutputProto output;
  // put in any custom data members that you need
};

TEST_F(GroundGearIntakeTest, CanPickupWithoutCurrentSpike) {
  SetGoal(c2017::ground_gear_intake::PICKUP);
  Update(20, false);
  EXPECT_TRUE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 12., 1e-5);
}

TEST_F(GroundGearIntakeTest, CanCarryWithoutCurrentSpike) {
  SetGoal(c2017::ground_gear_intake::CARRY);
  Update(20, false);
  EXPECT_FALSE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 0., 1e-5);
}

TEST_F(GroundGearIntakeTest, CanScoreWithoutCurrentSpike) {
  SetGoal(c2017::ground_gear_intake::SCORE);
  Update(20., false);
  EXPECT_FALSE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), -12., 1e-5);
}

TEST_F(GroundGearIntakeTest, CanScoreWithCurrentSpike) {
  SetGoal(c2017::ground_gear_intake::SCORE);
  Update(134., false);
  EXPECT_FALSE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), -12., 1e-5);
}

TEST_F(GroundGearIntakeTest, CanCarryAfterPickingUpStalls) {
  SetGoal(c2017::ground_gear_intake::PICKUP);
  Update(20., false);
  EXPECT_TRUE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 12., 1e-5);

  SetGoal(c2017::ground_gear_intake::PICKUP);
  Update(134., false);
  EXPECT_FALSE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 0., 1e-5);

  SetGoal(c2017::ground_gear_intake::CARRY);
  Update(20., false);
  EXPECT_FALSE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 0., 1e-5);
}

TEST_F(GroundGearIntakeTest, CanScoreAfterCarrying) {
  SetGoal(c2017::ground_gear_intake::CARRY);
  Update(20., false);
  EXPECT_FALSE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 0., 1e-5);

  SetGoal(c2017::ground_gear_intake::SCORE);
  Update(20., false);
  EXPECT_FALSE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), -12., 1e-5);
}

TEST_F(GroundGearIntakeTest, CanScoreAfterPickup) {
  SetGoal(c2017::ground_gear_intake::PICKUP);
  Update(20., false);
  EXPECT_TRUE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 12., 1e-5);

  SetGoal(c2017::ground_gear_intake::SCORE);
  Update(20., false);
  EXPECT_FALSE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), -12., 1e-5);
}

TEST_F(GroundGearIntakeTest, PickupStallCarryPickup) {
  SetGoal(c2017::ground_gear_intake::PICKUP);
  Update(20., false);
  EXPECT_TRUE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 12., 1e-5);

  SetGoal(c2017::ground_gear_intake::PICKUP);
  Update(134., false);
  EXPECT_FALSE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 0., 1e-5);

  SetGoal(c2017::ground_gear_intake::CARRY);
  Update(0., false);
  EXPECT_FALSE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 0., 1e-5);

  SetGoal(c2017::ground_gear_intake::PICKUP);
  Update(20., false);
  EXPECT_TRUE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 12., 1e-5);
}

TEST_F(GroundGearIntakeTest, DoesntTryToMoveWhenDisabled) {
  SetGoal(c2017::ground_gear_intake::PICKUP);
  Update(20., true);
  EXPECT_FALSE(GetIntakeState());
  EXPECT_EQ(GetVoltage(), 0);
}
