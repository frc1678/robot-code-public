#include "c2019/subsystems/superstructure/ground_hatch_intake/ground_hatch_intake.h"
#include "gtest/gtest.h"

namespace c2019 {
namespace ground_hatch_intake {

TEST(GroundHatchIntake, DropAndPickup) {
  GroundHatchIntake ground_hatch_intake;
  GroundHatchIntakeInputProto input;
  GroundHatchIntakeOutputProto output;
  GroundHatchIntakeStatusProto status;
  GroundHatchIntakeGoalProto goal;

  input->set_current(0);
  goal->set_goal(REQUEST_HATCH);

  ground_hatch_intake.SetGoal(goal);
  ground_hatch_intake.Update(input, &output, &status, true);

  EXPECT_EQ(status->state(), INTAKING);

  input->set_current(kCurrentThreshold + 1);
  goal->set_goal(NONE);
  ground_hatch_intake.SetGoal(goal);

  for (int i = 0; i < kPickupTicks + 1; i++) {
    ground_hatch_intake.SetGoal(goal);
    ground_hatch_intake.Update(input, &output, &status, true);

    EXPECT_EQ(status->state(), PICKING_UP);
  }

  ground_hatch_intake.SetGoal(goal);
  ground_hatch_intake.Update(input, &output, &status, true);

  EXPECT_EQ(status->state(), CARRYING);
}

}  // namespace ground_hatch_intake
}  // namespace c2019
