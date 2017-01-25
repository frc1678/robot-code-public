#include "ground_gear_intake.h"
#include "queue_types.h"
#include "gtest/gtest.h"

class GroundGearIntakeTest : public ::testing::Test {
 public:
  void Update(double current) {
    for (int i = 0; i < 1000; i++) {
      input->set_current(current);
      output = gear_intake.Update(input);
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
  c2017::ground_gear_intake::GroundGearIntakeOutputProto output;
  // put in any custom data members that you need
};

TEST_F(GroundGearIntakeTest, CanPickupWithoutCurrentSpike) {
  SetGoal(c2017::ground_gear_intake::PICKUP);
  Update(20);
  EXPECT_TRUE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 12., 1e-5);
}

TEST_F(GroundGearIntakeTest, CanCarryWithoutCurrentSpike) {
  SetGoal(c2017::ground_gear_intake::CARRY);
  Update(20);
  EXPECT_FALSE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 0., 1e-5);
}

TEST_F(GroundGearIntakeTest, CanScoreWithoutCurrentSpike) {
  SetGoal(c2017::ground_gear_intake::SCORE);
  Update(20.);
  EXPECT_FALSE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), -12., 1e-5);
}

TEST_F(GroundGearIntakeTest, CanScoreWithCurrentSpike) {
  SetGoal(c2017::ground_gear_intake::SCORE);
  Update(134.);
  EXPECT_FALSE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), -12., 1e-5);
}

TEST_F(GroundGearIntakeTest, CanCarryAfterPickingUpStalls) {
  SetGoal(c2017::ground_gear_intake::PICKUP);
  Update(20.);
  EXPECT_TRUE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 12., 1e-5);

  SetGoal(c2017::ground_gear_intake::PICKUP);
  Update(134.);
  EXPECT_FALSE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 0., 1e-5);

  SetGoal(c2017::ground_gear_intake::CARRY);
  Update(20.);
  EXPECT_FALSE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 0., 1e-5);
}

TEST_F(GroundGearIntakeTest, CanScoreAfterCarrying) {
  SetGoal(c2017::ground_gear_intake::CARRY);
  Update(20.);
  EXPECT_FALSE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 0., 1e-5);

  SetGoal(c2017::ground_gear_intake::SCORE);
  Update(20.);
  EXPECT_FALSE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), -12., 1e-5);
}

TEST_F(GroundGearIntakeTest, CanScoreAfterPickup) {
  SetGoal(c2017::ground_gear_intake::PICKUP);
  Update(20.);
  EXPECT_TRUE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 12., 1e-5);

  SetGoal(c2017::ground_gear_intake::SCORE);
  Update(20.);
  EXPECT_FALSE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), -12., 1e-5);
}

TEST_F(GroundGearIntakeTest, PickupStallCarryPickup) {
  SetGoal(c2017::ground_gear_intake::PICKUP);
  Update(20.);
  EXPECT_TRUE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 12., 1e-5);

  SetGoal(c2017::ground_gear_intake::PICKUP);
  Update(134.);
  EXPECT_FALSE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 0., 1e-5);

  SetGoal(c2017::ground_gear_intake::CARRY);
  Update(0.);
  EXPECT_FALSE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 0., 1e-5);

  SetGoal(c2017::ground_gear_intake::PICKUP);
  Update(20.);
  EXPECT_TRUE(GetIntakeState());
  EXPECT_NEAR(GetVoltage(), 12., 1e-5);
}
