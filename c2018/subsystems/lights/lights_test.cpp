#include "c2018/subsystems/lights/lights.h"
#include "gtest/gtest.h"

namespace c2018 {
namespace lights {

class LightsTest : public ::testing::Test {
 public:
  LightsTest() {}

  void Update() { lights_.Update(); }

  // PROTOS
  c2018::lights::LightsOutputProto output_proto_;
  c2018::score_subsystem::ScoreSubsystemStatusProto score_subsystem_status_proto_;

  void ReadMessages() {
    output_queue_.ReadLastMessage(&output_proto_);
    status_queue_->ReadLastMessage(&score_subsystem_status_proto_);
  }

  // QUEUES & READERS
  c2018::score_subsystem::ScoreSubsystemStatusQueue *status_queue_ =
      muan::queues::QueueManager<c2018::score_subsystem::ScoreSubsystemStatusProto>::Fetch();

  c2018::lights::LightsOutputQueue::QueueReader output_queue_ =
      muan::queues::QueueManager<c2018::lights::LightsOutputProto>::Fetch()
          ->MakeReader();

 private:
  c2018::lights::Lights lights_;
};

// TESTS
TEST_F(LightsTest, FlashLights) {
  score_subsystem_status_proto_->set_has_cube(true);
  status_queue_->WriteMessage(score_subsystem_status_proto_);

  Update();
  ReadMessages();

  EXPECT_TRUE(output_proto_->on());
}

TEST_F(LightsTest, FlashForCorrectTime) {
  int j = 0;
  score_subsystem_status_proto_->set_has_cube(true);

  for (int i = 0; i < kFlashTicks + 10; i++) {
    status_queue_->WriteMessage(score_subsystem_status_proto_);
    ReadMessages();
    Update();
    if (output_proto_->on()) {
      j++;
    }
  }

  EXPECT_NEAR(j, kFlashTicks / 2, 1);
}

}  // namespace lights
}  // namespace c2018
