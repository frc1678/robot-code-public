#include "catapult.h"
#include "gtest/gtest.h"
#include <iostream>

class CatapultTest : public ::testing::Test {
 public:
  CatapultTest() :
          input_queue_(std::make_shared<muan::queues::MessageQueue<CatapultInput, 10>>()),
          goal_queue_(std::make_shared<muan::queues::MessageQueue<CatapultGoal, 10>>()),
          output_queue_(std::make_shared<muan::queues::MessageQueue<CatapultOutput, 10>>()),
          status_queue_(std::make_shared<muan::queues::MessageQueue<CatapultStatus, 10>>()) {
    catapult_ = Catapult(input_queue_, goal_queue_, output_queue_, status_queue_);
    output_reader_ = output_queue_->MakeReader();
    status_reader_ = status_queue_->MakeReader();
  }

  void UpdateTest(CatapultGoal goal, Length distance_to_target) {
    goal_queue_->WriteMessage(goal);
    
    while(auto val = status_reader_.ReadMessage()) {
      status_ = *val;
    }

    CatapultInput input;
    input.scoop_pot = status_.scoop_angle;
    input.stop_pot = status_.stop_angle;
    input.stop_encoder = status_.stop_angle;
    input.distance_to_target = distance_to_target;
    input_queue_->WriteMessage(input);

    while(auto val = output_reader_.ReadMessage()) {
      output_ = *val;
    }
  }

  CatapultOutput output_;
  CatapultStatus status_;

 protected:
  Catapult catapult_;

  std::shared_ptr<muan::queues::MessageQueue<CatapultInput, 10>> input_queue_;
  std::shared_ptr<muan::queues::MessageQueue<CatapultGoal, 10>> goal_queue_;
  std::shared_ptr<muan::queues::MessageQueue<CatapultOutput, 10>> output_queue_;
  std::shared_ptr<muan::queues::MessageQueue<CatapultStatus, 10>> status_queue_;
  muan::queues::MessageQueue<CatapultOutput, 10>::QueueReader output_reader_;
  muan::queues::MessageQueue<CatapultStatus, 10>::QueueReader status_reader_;
};

TEST_F(CatapultTest, Calibrates) {
  std::cout << "got here" << std::endl;
  CatapultGoal goal;
  goal.should_shoot = true;
  goal.should_tuck = true;
  for(int i = 0; i < 400; i++) { 
    std::cout << "got here" << std::endl;
    UpdateTest(goal, 0 * m);
  }
  EXPECT_TRUE(status_.calibrated);
}
