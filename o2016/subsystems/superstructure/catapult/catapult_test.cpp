#include "catapult.h"
#include "gtest/gtest.h"
#include <iostream>

class CatapultTest : public ::testing::Test {
 public:
  CatapultTest() {}

  void UpdateTest(Length distance_to_target) {
    status_ = catapult_.get_status();
    output_ = catapult_.get_output();

    input_.set_scoop_pot(status_.scoop_angle());
    input_.set_stop_pot(status_.stop_angle());
    input_.set_stop_encoder(status_.stop_angle());
    input_.set_distance_to_target(distance_to_target);

    catapult_.Update(input_, goal_);
  }

  CatapultOutput output_;
  CatapultStatus status_;
  CatapultInput input_;
  CatapultGoal goal_;

 protected:
  Catapult catapult_;
};

TEST_F(CatapultTest, Calibrates) {
  goal_.set_goal(CatapultGoal::TUCK);
  input_.set_scoop_pot(0 * rad);
  input_.set_stop_pot(0 * rad);
  input_.set_stop_encoder(0 * rad);
  for(int i = 0; i < 400; i++) { 
    UpdateTest(0 * m);
  }
  EXPECT_TRUE(status_.calibrated());
}
