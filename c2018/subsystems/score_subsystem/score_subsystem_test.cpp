#include "test/gtest.h"
#include "c2018/subsystems/score_subsystems/"
#include "c2018/queuemanager/queuemanager.h"

class ScoreSubsystemTest : public ::testing::Test {
 public:
  ScoreSubsystemTest() {}
  void Update() { score_subsystem_.Update(); }

  void ReadMessages() {
    output_reader_.ReadLastMessage(&score_subsystem_output_proto_);
    status_reader_.ReadLastMessage(&score_subsystem_status_proto_);
  }

  void WriteMessages() {
    input_queue_->WriteMessage(climber_input_proto_);
    goal_reader_->WriteMessage(climber_goal_proto_);
    ds_status_reader_->WriteMessage(driver_station_proto_);
  }

  void SetGoals(bool batter_down, c2018::climber::Goal enum_goal, bool enabled) {
   score_subsystem_goal_proto_->set_put_down_batter(batter_down);
   score_subsystem_goal_proto_->set_climber_goal(enum_goal);
   driver_station_proto_->set_is_sys_active(enabled);
  }

 private:
  c2018::score_subsystem::ScoreSubsystem score_subsystem_;

  muan::queues::MessageQueue<c2018::score_subsystem::ScoreSubsystemInputProto>* input_queue_ =
      muan::queues::QueueManager<c2018::score_subsystem::ScoreSubsystemInputProto>::Fetch();
  muan::queues::MessageQueue<c2018::score_subsystem::ScoreSubsystemGoalProto>* goal_queue_ =
      muan::queues::QueueManager<c2018::score_subsystem::ScoreSubsystemInputProto>::Fetch();
  muan::queues::MessageQueue<c2018::score_subsystem::ScoreSubsystemStatusProto>::QueueReader status_reader_ =
      muan::queues::QueueManager<c2018::score_subsystem::ScoreSubsystemInputProto>::Fetch()->MakeReader();
  muan::queues::MessageQueue<c2018::score_subsystem::ScoreSubsystemOutputProto>::QueueReader output_reader_ =
      muan::queues::QueueManager<c2018::score_subsystem::ScoreSubsystemInputProto>::Fetch()->MakeReader();
};
