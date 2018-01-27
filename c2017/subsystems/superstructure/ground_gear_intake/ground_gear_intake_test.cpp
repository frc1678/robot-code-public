#include "c2017/subsystems/superstructure/ground_gear_intake/ground_gear_intake.h"
#include "c2017/subsystems/superstructure/ground_gear_intake/queue_types.h"
#include "gtest/gtest.h"

class GroundGearIntakeTest : public ::testing::Test {
 public:
  void Update(double current, bool robot_disabled, int num_ticks) {
    for (int i = 0; i < num_ticks; i++) {
      input->set_current(current);
      output = gear_intake.Update(input, !robot_disabled);
    }
  }

  void SetGoal(c2017::ground_gear_intake::Goal goal_state) {
    goal->set_goal(goal_state);
    gear_intake.SetGoal(goal);
  }

  double GetVoltage() { return output->roller_voltage(); }
  bool IsDown() { return output->intake_down(); }
  c2017::ground_gear_intake::State GetIntakeState() {
    return gear_intake.current_state();
  }

 private:
  c2017::ground_gear_intake::GroundGearIntake gear_intake;
  c2017::ground_gear_intake::GroundGearIntakeInputProto input;
  c2017::ground_gear_intake::GroundGearIntakeGoalProto goal;
  c2017::ground_gear_intake::GroundGearIntakeOutputProto output;
};

TEST_F(GroundGearIntakeTest, Idling) {
  SetGoal(c2017::ground_gear_intake::NONE);
  Update(0, false, 1);
  EXPECT_EQ(GetIntakeState(), c2017::ground_gear_intake::IDLE);
  EXPECT_EQ(GetVoltage(), 0.0);
}

TEST_F(GroundGearIntakeTest, Intaking) {
  SetGoal(c2017::ground_gear_intake::DROP);
  Update(0, false, 1);

  EXPECT_EQ(GetIntakeState(), c2017::ground_gear_intake::INTAKING);
  EXPECT_EQ(GetVoltage(), c2017::ground_gear_intake::kIntakeVoltage);

  SetGoal(c2017::ground_gear_intake::NONE);
  Update(0, false, 10);

  EXPECT_EQ(GetIntakeState(), c2017::ground_gear_intake::INTAKING);
  EXPECT_EQ(GetVoltage(), c2017::ground_gear_intake::kIntakeVoltage);
}

TEST_F(GroundGearIntakeTest, DroppingBallsWithGear) {
  SetGoal(c2017::ground_gear_intake::DROP);
  Update(70, false, 100);
  SetGoal(c2017::ground_gear_intake::START_DROPPING_BALLS);
  Update(70, false, 100);

  EXPECT_EQ(GetIntakeState(), c2017::ground_gear_intake::DROP_BALL_WITH_GEAR);
  EXPECT_EQ(GetVoltage(), c2017::ground_gear_intake::kPickupVoltage);
}

TEST_F(GroundGearIntakeTest, DroppingBallsWithoutGear) {
  SetGoal(c2017::ground_gear_intake::START_DROPPING_BALLS);
  Update(0, false, 100);
  EXPECT_EQ(GetIntakeState(),
            c2017::ground_gear_intake::DROP_BALL_WITHOUT_GEAR);
  EXPECT_EQ(GetVoltage(), 0);
}

TEST_F(GroundGearIntakeTest, FullSequence) {
  SetGoal(c2017::ground_gear_intake::DROP);
  Update(0, false, 1);

  SetGoal(c2017::ground_gear_intake::NONE);
  Update(15.0, false, 1);

  EXPECT_EQ(GetIntakeState(), c2017::ground_gear_intake::INTAKING);

  Update(70.0, false, 1);

  EXPECT_EQ(GetIntakeState(), c2017::ground_gear_intake::PICKING_UP);
  EXPECT_EQ(GetVoltage(), c2017::ground_gear_intake::kPickupVoltage);

  Update(15.0, false, 500);

  EXPECT_EQ(GetIntakeState(), c2017::ground_gear_intake::CARRYING);
  // 775pros will stall out at stall at voltages >= 2.5 if left indefinitely.
  EXPECT_EQ(GetVoltage(), c2017::ground_gear_intake::kCarryVoltage);

  Update(15.0, false, 100);

  EXPECT_EQ(GetIntakeState(), c2017::ground_gear_intake::CARRYING);

  SetGoal(c2017::ground_gear_intake::SCORE);
  Update(15.0, false, 1);

  EXPECT_EQ(GetIntakeState(), c2017::ground_gear_intake::SCORING);
  EXPECT_LT(GetVoltage(), 0.0);
}

TEST_F(GroundGearIntakeTest, OuttakeTest) {
  SetGoal(c2017::ground_gear_intake::OUTTAKE);
  Update(0, false, 1);
  EXPECT_EQ(GetIntakeState(), c2017::ground_gear_intake::OUTTAKING);
  EXPECT_EQ(GetVoltage(), c2017::ground_gear_intake::kOuttakeVoltage);
}

TEST_F(GroundGearIntakeTest, RiseFromIntaking) {
  SetGoal(c2017::ground_gear_intake::DROP);
  Update(0, false, 1);
  SetGoal(c2017::ground_gear_intake::RISE);
  Update(0, false, 1);
  EXPECT_EQ(GetIntakeState(), c2017::ground_gear_intake::IDLE);
}

TEST_F(GroundGearIntakeTest, NoRiseFromPickup) {
  SetGoal(c2017::ground_gear_intake::DROP);
  Update(90.0, false, 10);
  SetGoal(c2017::ground_gear_intake::RISE);
  Update(0, false, 1);
  EXPECT_EQ(GetIntakeState(), c2017::ground_gear_intake::PICKING_UP);
}
