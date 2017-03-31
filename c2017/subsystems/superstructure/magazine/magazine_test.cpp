#include "c2017/subsystems/superstructure/magazine/magazine.h"
#include "c2017/subsystems/superstructure/magazine/queue_types.h"
#include "gtest/gtest.h"

TEST(MagazineTest, CanExtendMagazine) {
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_IDLE);
  goal->set_magazine_extended(true);
  goal->set_side_goal(c2017::magazine::SideGoalState::SIDE_IDLE);

  magazine.SetGoal(goal);
  c2017::magazine::MagazineOutputProto output = magazine.Update(true);

  EXPECT_TRUE(output->magazine_extended());
}

TEST(MagazineTest, CanAgitateMagazine) {
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_IDLE);
  goal->set_magazine_extended(false);
  goal->set_side_goal(c2017::magazine::SideGoalState::SIDE_AGITATE);

  magazine.SetGoal(goal);
  c2017::magazine::MagazineOutputProto output = magazine.Update(true);

  EXPECT_EQ(output->side_voltage(), -6);
}

TEST(MagazineTest, UpperCanMove) {
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_FORWARD);
  goal->set_magazine_extended(false);
  goal->set_side_goal(c2017::magazine::SideGoalState::SIDE_IDLE);

  magazine.SetGoal(goal);
  c2017::magazine::MagazineOutputProto output = magazine.Update(true);

  EXPECT_EQ(output->upper_voltage(), 12);
}

TEST(MagazineTest, SendsNoVoltageWhenDisabled) {
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_upper_goal(c2017::magazine::UpperGoalState::UPPER_FORWARD);
  goal->set_magazine_extended(false);
  goal->set_side_goal(c2017::magazine::SideGoalState::SIDE_IDLE);

  magazine.SetGoal(goal);
  c2017::magazine::MagazineOutputProto output = magazine.Update(false);

  EXPECT_EQ(output->upper_voltage(), 0);
}
