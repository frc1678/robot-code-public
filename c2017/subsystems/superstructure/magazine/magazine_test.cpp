#include "c2017/subsystems/superstructure/magazine/magazine.h"
#include "c2017/subsystems/superstructure/magazine/queue_types.h"
#include "gtest/gtest.h"

TEST(MagazineTest, CanExtendMagazine) {
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::NONE);
  goal->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_IDLE);
  goal->set_score_gear(false);
  goal->set_magazine_extended(true);
  goal->set_side_goal(c2017::magazine::SideGoalState::SIDE_IDLE);

  magazine.SetGoal(goal);
  c2017::magazine::MagazineOutputProto output = magazine.Update(true);

  EXPECT_TRUE(output->magazine_extended());
}

TEST(MagazineTest, CanIntakeBoth) {
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::BOTH);
  goal->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_IDLE);
  goal->set_score_gear(false);
  goal->set_magazine_extended(true);
  goal->set_side_goal(c2017::magazine::SideGoalState::SIDE_IDLE);

  magazine.SetGoal(goal);
  c2017::magazine::MagazineOutputProto output = magazine.Update(true);

  EXPECT_FALSE(output->gear_intake_closed());

  output = magazine.Update(true);
}

TEST(MagazineTest, CanHPIntakeGear) {
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::GEAR);
  goal->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_IDLE);
  goal->set_score_gear(false);
  goal->set_magazine_extended(true);
  goal->set_side_goal(c2017::magazine::SideGoalState::SIDE_IDLE);

  magazine.SetGoal(goal);
  c2017::magazine::MagazineOutputProto output = magazine.Update(true);

  EXPECT_FALSE(output->gear_intake_closed());
}

TEST(MagazineTest, CanIntakeBalls) {
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::BALLS);
  goal->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_IDLE);
  goal->set_score_gear(false);
  goal->set_magazine_extended(true);
  goal->set_side_goal(c2017::magazine::SideGoalState::SIDE_IDLE);

  magazine.SetGoal(goal);
  c2017::magazine::MagazineOutputProto output = magazine.Update(true);

  EXPECT_TRUE(output->gear_intake_closed());
}

TEST(MagazineTest, CanAgitateMagazine) {
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::GEAR);
  goal->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_IDLE);
  goal->set_score_gear(false);
  goal->set_magazine_extended(false);
  goal->set_side_goal(c2017::magazine::SideGoalState::SIDE_AGITATE);

  magazine.SetGoal(goal);
  c2017::magazine::MagazineOutputProto output = magazine.Update(true);

  EXPECT_EQ(output->side_voltage(), -6);
}

TEST(MagazineTest, CanPullInBalls) {
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::GEAR);
  goal->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_IDLE);
  goal->set_score_gear(false);
  goal->set_magazine_extended(false);
  goal->set_side_goal(c2017::magazine::SideGoalState::SIDE_PULL_IN);

  magazine.SetGoal(goal);
  c2017::magazine::MagazineOutputProto output = magazine.Update(true);

  EXPECT_EQ(output->side_voltage(), 6);
}

TEST(MagazineTest, CanIntakeNothing) {
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::NONE);
  goal->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_IDLE);
  goal->set_score_gear(false);
  goal->set_magazine_extended(false);
  goal->set_side_goal(c2017::magazine::SideGoalState::SIDE_IDLE);

  magazine.SetGoal(goal);
  c2017::magazine::MagazineOutputProto output = magazine.Update(true);

  EXPECT_TRUE(output->gear_intake_closed());
}

TEST(MagazineTest, UpperCanMove) {
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::NONE);
  goal->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_FORWARD);
  goal->set_score_gear(false);
  goal->set_magazine_extended(false);
  goal->set_side_goal(c2017::magazine::SideGoalState::SIDE_IDLE);

  magazine.SetGoal(goal);
  c2017::magazine::MagazineOutputProto output = magazine.Update(true);

  EXPECT_EQ(output->upper_voltage(), 12);
}

TEST(MagazineTest, SendsNoVoltageWhenDisabled) {
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::NONE);
  goal->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_FORWARD);
  goal->set_score_gear(true);
  goal->set_magazine_extended(false);
  goal->set_side_goal(c2017::magazine::SideGoalState::SIDE_IDLE);

  magazine.SetGoal(goal);
  c2017::magazine::MagazineOutputProto output = magazine.Update(false);

  EXPECT_EQ(output->upper_voltage(), 0);
}
