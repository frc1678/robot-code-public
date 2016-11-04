#include "catapult.h"
#include "gtest/gtest.h"
#include <iostream>

using namespace muan::units;
using namespace o2016::catapult;

class CatapultTest : public ::testing::Test {
 public:
  CatapultTest() :
    output_reader_(::o2016::QueueManager::GetInstance().catapult_output_queue().MakeReader()),
    status_reader_(::o2016::QueueManager::GetInstance().catapult_status_queue().MakeReader()) {}

  void UpdateTest(Length distance_to_target) {
    status_ = *(status_reader_.ReadLastMessage());
    output_ = *(output_reader_.ReadLastMessage());

    input_->set_scoop_pot(status_->scoop_angle());
    input_->set_hardstop_pot(status_->hardstop_angle());
    // input_.set_distance_to_target(distance_to_target);

    ::o2016::QueueManager::GetInstance().catapult_input_queue().WriteMessage(input_);
    ::o2016::QueueManager::GetInstance().catapult_goal_queue().WriteMessage(goal_);
    catapult_.Update();
  }

  CatapultOutputProto output_;
  CatapultStatusProto status_;
  CatapultInputProto input_;
  CatapultGoalProto goal_;

 protected:
  CatapultOutputQueue::QueueReader output_reader_;
  CatapultStatusQueue::QueueReader status_reader_;
  Catapult catapult_;
};

TEST_F(CatapultTest, Terminates) {
  input_->set_scoop_pot(0 * rad);
  input_->set_hardstop_pot(0 * rad);
  for(int i = 0; i < 3; i++) {
    goal_->set_goal(CatapultGoal::INTAKE);
    for(int i = 0; i < 400; i++) {
      UpdateTest(0 * m);
    }
    EXPECT_TRUE(status_->at_goal());
    goal_->set_goal(CatapultGoal::PREP_SHOT);
    for(int i = 0; i < 400; i++) {
      UpdateTest(0 * m);
    }
    EXPECT_TRUE(status_->at_goal());
  }
}
