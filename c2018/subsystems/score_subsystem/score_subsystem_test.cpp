#include "gtest/gtest.h"
#include "c2018/subsystems/score_subsystem/score_subsystem.h"
#include "muan/queues/queue_manager.h"

class ScoreSubsystemTest : public ::testing::Test {
 public:
  ScoreSubsystemTest() {}
  void Update() { score_subsystem_.Update(); }

  void ReadMessages() {
    score_subsystem_output_queue_.ReadLastMessage(&score_subsystem_output_proto_);
    score_subsystem_status_queue_.ReadLastMessage(&score_subsystem_status_proto_);
  }

  void WriteMessages() {
    score_subsystem_input_queue_->WriteMessage(score_subsystem_input_proto_);
    score_subsystem_goal_queue_->WriteMessage(score_subsystem_goal_proto_);
    driver_station_queue_->WriteMessage(driver_station_proto_);
  }

  void SetGoals(c2018::score_subsystem::ScoreGoal score_goal, bool outputs_enabled) {
   score_subsystem_goal_proto_->set_score_goal(score_goal);
   driver_station_proto_->set_is_sys_active(outputs_enabled);
  }

  void SetInput(double elevator_encoder, bool elevator_hall, double wrist_encoder, bool wrist_hall) {
    score_subsystem_input_proto_->set_elevator_encoder(elevator_encoder);
    score_subsystem_input_proto_->set_wrist_encoder(wrist_encoder);
    score_subsystem_input_proto_->set_elevator_hall(elevator_hall);
    score_subsystem_input_proto_->set_wrist_hall(wrist_hall);
  }

  bool outputs_enabled_;

  muan::wpilib::DriverStationQueue* driver_station_queue_ =
      muan::queues::QueueManager<muan::wpilib::DriverStationProto>::Fetch();

  c2018::score_subsystem::ScoreSubsystemGoalQueue* score_subsystem_goal_queue_ =
      muan::queues::QueueManager<c2018::score_subsystem::ScoreSubsystemGoalProto>::Fetch();

  c2018::score_subsystem::ScoreSubsystemInputQueue* score_subsystem_input_queue_ =
      muan::queues::QueueManager<c2018::score_subsystem::ScoreSubsystemInputProto>::Fetch();

  c2018::score_subsystem::ScoreSubsystemStatusQueue::QueueReader score_subsystem_status_queue_ =
      muan::queues::QueueManager<c2018::score_subsystem::ScoreSubsystemStatusProto>::Fetch()
          ->MakeReader();

  c2018::score_subsystem::ScoreSubsystemOutputQueue::QueueReader score_subsystem_output_queue_ =
      muan::queues::QueueManager<c2018::score_subsystem::ScoreSubsystemOutputProto>::Fetch()
          ->MakeReader();

  muan::wpilib::DriverStationProto driver_station_proto_;
  c2018::score_subsystem::ScoreSubsystemGoalProto score_subsystem_goal_proto_;
  c2018::score_subsystem::ScoreSubsystemInputProto score_subsystem_input_proto_;
  c2018::score_subsystem::ScoreSubsystemStatusProto score_subsystem_status_proto_;
  c2018::score_subsystem::ScoreSubsystemOutputProto score_subsystem_output_proto_;

 private:
  c2018::score_subsystem::ScoreSubsystem score_subsystem_;
};

