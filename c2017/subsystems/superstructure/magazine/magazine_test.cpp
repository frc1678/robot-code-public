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

TEST(MagazineTest, CanIntakeBoth) {
  c2017::magazine::MagazineInputProto input;
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::BOTH);
  goal->set_conveyor_goal(c2017::magazine::ConveyorGoalState::CONVEYOR_IDLE);
  goal->set_brush_goal(c2017::magazine::BrushGoalState::BRUSH_IDLE);
  goal->set_score_gear(false);
  goal->set_rotate_gear(true);
  goal->set_magazine_extended(true);

  input->set_has_hp_gear(false);

  magazine.SetGoal(goal);
  c2017::magazine::MagazineOutputProto output = magazine.Update(input);

  EXPECT_FALSE(output->gear_intake_covered());

  input->set_has_hp_gear(true);

  output = magazine.Update(input);

  EXPECT_TRUE(output->gear_intake_covered());
}

TEST(MagazineTest, CanIntakeGear) {
  c2017::magazine::MagazineInputProto input;
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::GEAR);
  goal->set_conveyor_goal(c2017::magazine::ConveyorGoalState::CONVEYOR_IDLE);
  goal->set_brush_goal(c2017::magazine::BrushGoalState::BRUSH_IDLE);
  goal->set_score_gear(false);
  goal->set_rotate_gear(true);
  goal->set_magazine_extended(true);
  input->set_has_hp_gear(false);

  magazine.SetGoal(goal);
  c2017::magazine::MagazineOutputProto output = magazine.Update(input);

  EXPECT_FALSE(output->gear_intake_covered());
}

TEST(MagazineTest, CanIntakeBalls) {
  c2017::magazine::MagazineInputProto input;
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::BALLS);
  goal->set_conveyor_goal(c2017::magazine::ConveyorGoalState::CONVEYOR_IDLE);
  goal->set_brush_goal(c2017::magazine::BrushGoalState::BRUSH_IDLE);
  goal->set_score_gear(false);
  goal->set_rotate_gear(true);
  goal->set_magazine_extended(true);
  input->set_has_hp_gear(false);

  magazine.SetGoal(goal);
  c2017::magazine::MagazineOutputProto output = magazine.Update(input);

  EXPECT_TRUE(output->gear_intake_covered());
}

TEST(MagazineTest, CanRotateGear) {
  c2017::magazine::MagazineInputProto input;
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_conveyor_goal(c2017::magazine::ConveyorGoalState::CONVEYOR_IDLE);
  goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::GEAR);
  goal->set_brush_goal(c2017::magazine::BrushGoalState::BRUSH_IDLE);
  goal->set_score_gear(false);
  goal->set_rotate_gear(true);
  goal->set_magazine_extended(false);

  magazine.SetGoal(goal);
  c2017::magazine::MagazineOutputProto output = magazine.Update(input);

  EXPECT_EQ(output->gear_rotator_voltage(), 3);
}

TEST(MagazineTest, CanScoreGearWhileRotating) {
  c2017::magazine::MagazineInputProto input;
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_conveyor_goal(c2017::magazine::ConveyorGoalState::CONVEYOR_IDLE);
  goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::GEAR);
  goal->set_brush_goal(c2017::magazine::BrushGoalState::BRUSH_IDLE);
  goal->set_score_gear(true);
  goal->set_rotate_gear(true);
  goal->set_magazine_extended(false);

  magazine.SetGoal(goal);
  c2017::magazine::MagazineOutputProto output = magazine.Update(input);

  EXPECT_EQ(output->gear_rotator_voltage(), 3);
  EXPECT_TRUE(output->score_gear());
}

TEST(MagazineTest, CanIntakeNothing) {
  c2017::magazine::MagazineInputProto input;
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_conveyor_goal(c2017::magazine::ConveyorGoalState::CONVEYOR_IDLE);
  goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::NONE);
  goal->set_brush_goal(c2017::magazine::BrushGoalState::BRUSH_IDLE);
  goal->set_score_gear(false);
  goal->set_rotate_gear(true);
  goal->set_magazine_extended(false);

  magazine.SetGoal(goal);
  c2017::magazine::MagazineOutputProto output = magazine.Update(input);

  EXPECT_FALSE(output->magazine_extended());
}

TEST(MagazineTest, ConveyorCanMoveForwardThenBackward) {
  c2017::magazine::MagazineInputProto input;
  c2017::magazine::MagazineGoalProto goal;
  c2017::magazine::Magazine magazine;

  goal->set_conveyor_goal(c2017::magazine::ConveyorGoalState::CONVEYOR_FORWARD);
  goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::NONE);
  goal->set_brush_goal(c2017::magazine::BrushGoalState::BRUSH_IDLE);
  goal->set_score_gear(false);
  goal->set_rotate_gear(true);
  goal->set_magazine_extended(false);

  magazine.SetGoal(goal);
  c2017::magazine::MagazineOutputProto output = magazine.Update(input);

  EXPECT_EQ(output->conveyor_voltage(), 12);

  goal->set_conveyor_goal(c2017::magazine::ConveyorGoalState::CONVEYOR_BACKWARD);
  goal->set_hp_intake_goal(c2017::magazine::HPIntakeGoalState::NONE);
  goal->set_brush_goal(c2017::magazine::BrushGoalState::BRUSH_IDLE);
  goal->set_score_gear(false);
  goal->set_rotate_gear(true);
  goal->set_magazine_extended(false);

  magazine.SetGoal(goal);
  output = magazine.Update(input);

  EXPECT_EQ(output->conveyor_voltage(), -12);
}
