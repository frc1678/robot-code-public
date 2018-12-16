#include <cmath>
#include "gtest/gtest.h"
#include "muan/lime/lime.h"

namespace muan {
namespace lime {

class LimeTest : public ::testing::Test {
 public:
  void Update() {
    WriteMessages();
    lime_.Update();
    ReadMessages();
  }
  void WriteMessages() { lime_goal_queue_->WriteMessage(lime_goal_proto_); }
  void ReadMessages() {
    lime_status_reader_.ReadLastMessage(&lime_status_proto_);
  }
  void RunFor(int ticks) {
    for (int i = 0; i < ticks; i++) {
      Update();
    }
  }
  void SetGoal(LightState state) { lime_goal_proto_->set_light_state(state); }
  LimeStatusProto lime_status_proto_;
  LimeGoalProto lime_goal_proto_;

 protected:
  Lime lime_{1.0, 30, 0.5};

 private:
  LimeStatusQueue::QueueReader lime_status_reader_{
      muan::queues::QueueManager<LimeStatusProto>::Fetch()->MakeReader()};

  LimeGoalQueue* lime_goal_queue_{
      muan::queues::QueueManager<LimeGoalProto>::Fetch()};
};
// TESTS
TEST_F(LimeTest, CubeDistance) {
  double result = lime_.ObjectDistance(20);  // Object is at a 20 degree angle
                                             // with respect to the limelight,
                                             // whose position is set in the
                                             // test fixture class

  double expected = (1.0 - 0.5) * (std::tan((30 + 20) * (M_PI / 180.)));
  // Simple trig, one leg of triangle is the offset
  // from the limelight height to the object height,
  // and the angle is the object angle with respect to
  // the limelight + the limelight offset angle

  EXPECT_NEAR(result, expected, 1e-3);
}

TEST_F(LimeTest, HasNoTarget) {
  lime_.target_present_ = false;
  Update();
  EXPECT_FALSE(lime_status_proto_->has_dist());
  EXPECT_FALSE(lime_status_proto_->has_theta());
  EXPECT_FALSE(lime_status_proto_->has_relative_x());
  EXPECT_FALSE(lime_status_proto_->has_relative_y());
}

}  // namespace lime
}  // namespace muan
