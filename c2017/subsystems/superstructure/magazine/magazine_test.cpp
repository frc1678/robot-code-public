#include "c2017/subsystems/superstructure/magazine/magazine.h"
#include "c2017/subsystems/superstructure/magazine/queue_types.h"
#include "gtest/gtest.h"

/*
class MagazineTest : public ::testing::Test {
 public:
  void Update(c2017::magazine::MagazineInputProto input) {
    magazine.Update(input);
  }

  SetGoal(goal) {
    magazine.SetGoal(goal);
  }

  void SetConveyorCurrent(double conveyor);


 private:
  c2017::magazine::Magazine magazine;
  
}; */


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
