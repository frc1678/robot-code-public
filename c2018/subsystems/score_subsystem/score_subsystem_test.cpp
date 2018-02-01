#include "gtest/gtest.h"
#include "c2018/subsystems/score_subsystem/score_subsystem.h"
#include "muan/queues/queue_manager.h"

class ScoreSubsystemTest : public ::testing::Test {
 public:
  ScoreSubsystemTest() {}
  void Update() { score_subsystem_.Update(); }

  void ReadMessages() {
    output_reader_.ReadLastMessage(&score_subsystem_output_proto_);
    status_reader_.ReadLastMessage(&score_subsystem_status_proto_);
  }

  void WriteMessages() {
    input_queue_->WriteMessage(score_subsystem_input_proto_);
    goal_reader_->WriteMessage(score_subsystem_goal_proto_);
    ds_status_reader_->WriteMessage(driver_station_proto_);
  }

  void SetGoals(ElevatorHeight elevator_height, ClawMode claw_mode, IntakeMode intake_mode, double elevator_velocity, double intake_voltage) {
   score_subsystem_goal_proto_->set_elevator_height(batter_down);
   score_subsystem_goal_proto_->set_climber_goal(enum_goal);
   driver_station_proto_->set_is_sys_active(enabled);
  }



  void SetInput(double elevator_encoder, elevator_hall, wrist_encoder, wrist_hall) { score_subsystem_input_proto_->set_position(position); }
 private:
  c2018::score_subsystem::ScoreSubsystem score_subsystem_;

  muan::wpilib::DriverStationQueue* driver_station_queue_ = 
      muan::queues::QueueReader<muan::wpilib::DriverStationProto>::Fetch();

  muan::queues::MessageQueue<c2018::score_subsystem::ScoreSubsystemInputProto>* input_queue_ =
      muan::queues::QueueManager<c2018::score_subsystem::ScoreSubsystemInputProto>::Fetch();

  muan::queues::MessageQueue<c2018::score_subsystem::ScoreSubsystemGoalProto>* goal_queue_ =
      muan::queues::QueueManager<c2018::score_subsystem::ScoreSubsystemInputProto>::Fetch();

  muan::queues::MessageQueue<c2018::score_subsystem::ScoreSubsystemStatusProto>::QueueReader status_reader_ =
      muan::queues::QueueManager<c2018::score_subsystem::ScoreSubsystemInputProto>::Fetch()->MakeReader();

  muan::queues::MessageQueue<c2018::score_subsystem::ScoreSubsystemOutputProto>::QueueReader output_reader_ =
      muan::queues::QueueManager<c2018::score_subsystem::ScoreSubsystemInputProto>::Fetch()->MakeReader();

  //PROTOS
  muan::wpilib::DriverStationProto driver_station_proto_;
  c2018::score_subsystem::ScoreSubsystemGoalProto score-subsystem_goal_proto_;
  c2018::score_subsystem::ScoreSubsystemInputProto score_subsystem_input_proto_;
  c2018::score_subsystem::ScoreSubsystemStatusProto score_subsystem_status_proto_;
  c2018::score_subsystem::ScoreSubsystemOutputProto score_subsystem_output_proto_;
};

TEST_F(ScoreSubsystemTest, Idle) {
  SetGoals(c2018::score_subsystem::ElevatorHeight::HEIGHT_0, );
  SetInput();

}
