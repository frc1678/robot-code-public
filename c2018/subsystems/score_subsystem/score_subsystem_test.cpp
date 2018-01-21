#include "test/gtest.h"
#include "c2018/subsystems/score_subsystems/"
#include "c2018/queuemanager/queuemanager.h"

class ScoreSubsystemTest : public ::testing::Test {
 public:
  ScoreSubsystemTest() {}
  void Update() { score_subsystem_.Update(); }

  void ReadMessages() {


 private:
  c2018::score_subsystem::ScoreSubsystem score_subsystem_;
  muan::queues::MessageQueue<c2018::score_subsystem::ScoreSubsystemInputProto>* input_queue_
  c2018::score_subsystem::ScoreSubsystemGoalQueue* score_subsystem_qoal_queue_ =
  		muan::queues::QueueManager<c2018::score_subsystem::ScoreSubsystemGoalProto>::Fetch();
  c2018::score_subsystem::ScoreSubsystemInputQueue
};
