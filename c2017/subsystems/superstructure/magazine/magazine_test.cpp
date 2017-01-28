#include "c2017/subsystems/superstructure/magazine/magazine.h"
#include "c2017/subsystems/superstructure/magazine/queue_types.h"
#include "gtest/gtest.h"

TEST(MagazineTest, CanExtendMagazine) {
  c2017::magazine::MagazineInputProto input;
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_conveyor_goal(c2017::magazine::ConveyorGoalState::CONVEYOR_IDLE);
  goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::NONE);
  goal->set_brush_goal(c2017::magazine::BrushGoalState::BRUSH_IDLE);
  goal->set_score_gear(false);
  goal->set_rotate_gear(true);
  goal->set_magazine_extended(true);

  magazine.SetGoal(goal);
  c2017::magazine::MagazineOutputProto output = magazine.Update(input);
  EXPECT_TRUE(output->magazine_extended());
}

TEST(MagazineTest, CantIntakeBoth) {
  c2017::magazine::MagazineInputProto input;
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::BOTH);
  input->set_has_hp_gear(true);
  
  c2017::magazine::MagazineOutputProto output = magazine.Update(input);
  magazine.SetGoal(goal);

  EXPECT_TRUE(output->gear_intake_covered());
}
