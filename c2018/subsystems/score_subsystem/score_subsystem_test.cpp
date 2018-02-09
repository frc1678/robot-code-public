#include "c2018/subsystems/score_subsystem/score_subsystem.h"
#include "gtest/gtest.h"
#include "muan/queues/queue_manager.h"

class ScoreSubsystemTest : public ::testing::Test {
 public:
  ScoreSubsystemTest() {
    wrist_plant_ = muan::control::StateSpacePlant<1, 3, 1>(
        frc1678::wrist_controller::controller::A(),
        frc1678::wrist_controller::controller::B(),
        frc1678::wrist_controller::controller::C());

    elevator_plant_ = muan::control::StateSpacePlant<1, 3, 1>(
        frc1678::elevator_controller::controller::first_stage_integral::A(),
        frc1678::elevator_controller::controller::first_stage_integral::B(),
        frc1678::elevator_controller::controller::first_stage_integral::C());
  }

  void Update() {
    if (elevator_plant_.x(0) < 0) {
      elevator_plant_.x(0) = 0;
    }

    if (wrist_plant_.x(0) < 0) {
      wrist_plant_.x(0) = 0;
    }

    score_subsystem_input_proto_->set_elevator_hall(
        elevator_plant_.x(0) >= 0.88 && elevator_plant_.x(0) <= 0.92);
    score_subsystem_input_proto_->set_wrist_hall(elevator_plant_.x(0) >= 0.21 &&
                                                 wrist_plant_.x(0) <= 0.25);

    WriteMessages();

    score_subsystem_.Update();

    SetWeights(elevator_plant_.x()(0, 0) >= 1.0,
               score_subsystem_input_proto_->has_cube());

    elevator_plant_.Update(
        (Eigen::Matrix<double, 1, 1>()
         << score_subsystem_output_proto_->elevator_voltage())
            .finished());
    wrist_plant_.Update((Eigen::Matrix<double, 1, 1>()
                         << score_subsystem_output_proto_->wrist_voltage())
                            .finished());

    ReadMessages();

    LogicCheck();
  }

  void LogicCheck() {
    if (score_subsystem_status_proto_->elevator_actual_height() < 0.89 ||
        score_subsystem_status_proto_->elevator_unprofiled_goal() < 0.89) {
      EXPECT_TRUE(score_subsystem_status_proto_->wrist_unprofiled_goal() <=
                  M_PI / 2);
    }

    if (score_subsystem_status_proto_->wrist_angle() > M_PI / 2) {
      EXPECT_NEAR(score_subsystem_status_proto_->elevator_unprofiled_goal(),
                  muan::utils::Cap(
                      score_subsystem_status_proto_->elevator_unprofiled_goal(),
                      0.9, 2),
                  1e-3);
    }

    EXPECT_NEAR(score_subsystem_output_proto_->elevator_voltage(), 0, 12);
    EXPECT_NEAR(score_subsystem_output_proto_->wrist_voltage(), 0, 12);
  }

  void ReadMessages() {
    score_subsystem_output_queue_.ReadLastMessage(
        &score_subsystem_output_proto_);
    score_subsystem_status_queue_.ReadLastMessage(
        &score_subsystem_status_proto_);
  }

  void WriteMessages() {
    score_subsystem_input_queue_->WriteMessage(score_subsystem_input_proto_);
    score_subsystem_goal_queue_->WriteMessage(score_subsystem_goal_proto_);
    driver_station_queue_->WriteMessage(driver_station_proto_);
  }

  void SetGoal(c2018::score_subsystem::ScoreGoal score_goal,
               bool outputs_enabled) {
    score_subsystem_goal_proto_->set_score_goal(score_goal);
    driver_station_proto_->set_is_sys_active(outputs_enabled);
  }

  void SetInput(double elevator_encoder, bool elevator_hall,
                double wrist_encoder, bool wrist_hall) {
    score_subsystem_input_proto_->set_elevator_encoder(elevator_encoder);
    score_subsystem_input_proto_->set_wrist_encoder(wrist_encoder);
    score_subsystem_input_proto_->set_elevator_hall(elevator_hall);
    score_subsystem_input_proto_->set_wrist_hall(wrist_hall);
  }

  void SetLoopInput(double elevator_encoder, double wrist_encoder) {
    score_subsystem_input_proto_->set_elevator_encoder(elevator_encoder);
    score_subsystem_input_proto_->set_wrist_encoder(wrist_encoder);
  }

  bool outputs_enabled_;

  muan::wpilib::DriverStationQueue* driver_station_queue_ =
      muan::queues::QueueManager<muan::wpilib::DriverStationProto>::Fetch();

  c2018::score_subsystem::ScoreSubsystemGoalQueue* score_subsystem_goal_queue_ =
      muan::queues::QueueManager<
          c2018::score_subsystem::ScoreSubsystemGoalProto>::Fetch();

  c2018::score_subsystem::ScoreSubsystemInputQueue*
      score_subsystem_input_queue_ = muan::queues::QueueManager<
          c2018::score_subsystem::ScoreSubsystemInputProto>::Fetch();

  c2018::score_subsystem::ScoreSubsystemStatusQueue::QueueReader
      score_subsystem_status_queue_ =
          muan::queues::QueueManager<
              c2018::score_subsystem::ScoreSubsystemStatusProto>::Fetch()
              ->MakeReader();

  c2018::score_subsystem::ScoreSubsystemOutputQueue::QueueReader
      score_subsystem_output_queue_ =
          muan::queues::QueueManager<
              c2018::score_subsystem::ScoreSubsystemOutputProto>::Fetch()
              ->MakeReader();

  muan::wpilib::DriverStationProto driver_station_proto_;
  c2018::score_subsystem::ScoreSubsystemGoalProto score_subsystem_goal_proto_;
  c2018::score_subsystem::ScoreSubsystemInputProto score_subsystem_input_proto_;
  c2018::score_subsystem::ScoreSubsystemStatusProto
      score_subsystem_status_proto_;
  c2018::score_subsystem::ScoreSubsystemOutputProto
      score_subsystem_output_proto_;

 protected:
  muan::control::StateSpacePlant<1, 3, 1> elevator_plant_;
  muan::control::StateSpacePlant<1, 3, 1> wrist_plant_;

 private:
  c2018::score_subsystem::ScoreSubsystem score_subsystem_;

  void SetWeights(bool second_stage, bool has_cube) {
    if (second_stage && has_cube) {
      elevator_plant_.A() = frc1678::elevator_controller::controller::
          second_stage_cube_integral::A();
      elevator_plant_.B() = frc1678::elevator_controller::controller::
          second_stage_cube_integral::B();
      elevator_plant_.C() = frc1678::elevator_controller::controller::
          second_stage_cube_integral::C();
    } else if (second_stage && !has_cube) {
      elevator_plant_.A() =
          frc1678::elevator_controller::controller::second_stage_integral::A();
      elevator_plant_.B() =
          frc1678::elevator_controller::controller::second_stage_integral::B();
      elevator_plant_.C() =
          frc1678::elevator_controller::controller::second_stage_integral::C();
    } else if (!second_stage && has_cube) {
      elevator_plant_.A() = frc1678::elevator_controller::controller::
          first_stage_cube_integral::A();
      elevator_plant_.B() = frc1678::elevator_controller::controller::
          first_stage_cube_integral::B();
      elevator_plant_.C() = frc1678::elevator_controller::controller::
          first_stage_cube_integral::C();
    } else if (!second_stage && !has_cube) {
      elevator_plant_.A() =
          frc1678::elevator_controller::controller::first_stage_integral::A();
      elevator_plant_.B() =
          frc1678::elevator_controller::controller::first_stage_integral::B();
      elevator_plant_.C() =
          frc1678::elevator_controller::controller::first_stage_integral::C();
    }
  }
};

TEST_F(ScoreSubsystemTest, Disabled) {
  SetGoal(c2018::score_subsystem::ScoreGoal::HEIGHT_2, false);
  SetInput(0, false, 0, false);

  Update();

  EXPECT_EQ(score_subsystem_output_proto_->elevator_voltage(), 0);
  EXPECT_EQ(score_subsystem_output_proto_->intake_voltage(), 0);
  EXPECT_EQ(score_subsystem_output_proto_->wrist_voltage(), 0);
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_open());
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_close());
}

TEST_F(ScoreSubsystemTest, FirstCube) {
  SetGoal(c2018::score_subsystem::ScoreGoal::HEIGHT_0, true);
  SetInput(0, false, 0, false);

  for (int i = 0; i < 1200; i++) {
    SetLoopInput(elevator_plant_.y(0), wrist_plant_.y(0));
    Update();
  }

  EXPECT_NEAR(score_subsystem_status_proto_->elevator_actual_height(),
              c2018::score_subsystem::kElevatorFirstCube, 1e-3);
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_open());
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_close());
}

TEST_F(ScoreSubsystemTest, SecondCube) {
  SetGoal(c2018::score_subsystem::ScoreGoal::HEIGHT_1, true);
  SetInput(0, false, 0, false);

  for (int i = 0; i < 1200; i++) {
    SetLoopInput(elevator_plant_.y(0), wrist_plant_.y(0));
    Update();
  }

  EXPECT_NEAR(score_subsystem_status_proto_->elevator_actual_height(),
              c2018::score_subsystem::kElevatorSecondCube, 1e-3);
  EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(), 0, 1e-3);
  EXPECT_NEAR(score_subsystem_output_proto_->intake_voltage(), 12, 1e-3);
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_open());
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_close());
}

TEST_F(ScoreSubsystemTest, ThirdCube) {
  SetGoal(c2018::score_subsystem::ScoreGoal::HEIGHT_2, true);
  SetInput(0, false, 0, false);

  for (int i = 0; i < 1200; i++) {
    SetLoopInput(elevator_plant_.y(0), wrist_plant_.y(0));
    Update();
  }

  EXPECT_NEAR(score_subsystem_status_proto_->elevator_actual_height(),
              c2018::score_subsystem::kElevatorThirdCube, 1e-3);
  EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(), 0, 1e-3);
  EXPECT_NEAR(score_subsystem_output_proto_->intake_voltage(), 12, 1e-3);
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_open());
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_close());
}

TEST_F(ScoreSubsystemTest, PrepLowScore) {
  SetGoal(c2018::score_subsystem::ScoreGoal::PREP_SCORE_LOW, true);
  SetInput(0, false, 0, false);

  for (int i = 0; i < 1200; i++) {
    SetLoopInput(elevator_plant_.y(0), wrist_plant_.y(0));
    Update();
  }

  EXPECT_NEAR(score_subsystem_status_proto_->elevator_actual_height(),
              c2018::score_subsystem::kElevatorScoreLow, 1e-3);
  EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(), 0, 1e-3);
  EXPECT_NEAR(score_subsystem_output_proto_->intake_voltage(), 0, 1e-3);
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(score_subsystem_output_proto_->wrist_solenoid_close());
}

TEST_F(ScoreSubsystemTest, PrepMidScore) {
  SetGoal(c2018::score_subsystem::ScoreGoal::PREP_SCORE_MID, true);
  SetInput(0, false, 0, false);

  for (int i = 0; i < 1200; i++) {
    SetLoopInput(elevator_plant_.y(0), wrist_plant_.y(0));
    Update();
  }

  EXPECT_NEAR(score_subsystem_status_proto_->elevator_actual_height(),
              c2018::score_subsystem::kElevatorScoreMid, 1e-3);
  EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(), 0, 1e-3);
  EXPECT_NEAR(score_subsystem_output_proto_->intake_voltage(), 0, 1e-3);
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(score_subsystem_output_proto_->wrist_solenoid_close());
}

TEST_F(ScoreSubsystemTest, PrepHighScore) {
  SetGoal(c2018::score_subsystem::ScoreGoal::PREP_SCORE_HIGH, true);
  SetInput(0, false, 0, false);

  for (int i = 0; i < 1200; i++) {
    SetLoopInput(elevator_plant_.y(0), wrist_plant_.y(0));
    Update();
  }

  EXPECT_NEAR(score_subsystem_status_proto_->elevator_actual_height(),
              c2018::score_subsystem::kElevatorScoreHigh, 1e-3);
  EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(), 0, 1e-3);
  EXPECT_NEAR(score_subsystem_output_proto_->intake_voltage(), 0, 1e-3);
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(score_subsystem_output_proto_->wrist_solenoid_close());
}

TEST_F(ScoreSubsystemTest, LowScore) {
  SetGoal(c2018::score_subsystem::ScoreGoal::SCORE_LOW, true);
  SetInput(0, false, 0, false);

  for (int i = 0; i < 1200; i++) {
    SetLoopInput(elevator_plant_.y(0), wrist_plant_.y(0));
    Update();
  }

  EXPECT_NEAR(score_subsystem_status_proto_->elevator_actual_height(),
              c2018::score_subsystem::kElevatorScoreLow, 1e-3);
  EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(), 0, 1e-3);
  EXPECT_NEAR(score_subsystem_output_proto_->intake_voltage(), -12, 1e-3);
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_open());
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_close());
}

TEST_F(ScoreSubsystemTest, MidScore) {
  SetGoal(c2018::score_subsystem::ScoreGoal::SCORE_MID, true);
  SetInput(0, false, 0, false);

  for (int i = 0; i < 1200; i++) {
    SetLoopInput(elevator_plant_.y(0), wrist_plant_.y(0));
    Update();
  }

  EXPECT_NEAR(score_subsystem_status_proto_->elevator_actual_height(),
              c2018::score_subsystem::kElevatorScoreMid, 1e-3);
  EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(), 0, 1e-3);
  EXPECT_NEAR(score_subsystem_output_proto_->intake_voltage(), -12, 1e-3);
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_open());
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_close());
}

TEST_F(ScoreSubsystemTest, HighScore) {
  SetGoal(c2018::score_subsystem::ScoreGoal::SCORE_HIGH, true);
  SetInput(0, false, 0, false);

  for (int i = 0; i < 1200; i++) {
    SetLoopInput(elevator_plant_.y(0), wrist_plant_.y(0));
    Update();
  }

  EXPECT_NEAR(score_subsystem_status_proto_->elevator_actual_height(),
              c2018::score_subsystem::kElevatorScoreHigh, 1e-3);
  EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(), 0, 1e-3);
  EXPECT_NEAR(score_subsystem_output_proto_->intake_voltage(), -12, 1e-3);
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_open());
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_close());
}

TEST_F(ScoreSubsystemTest, PrepMidScoreBack) {
  SetGoal(c2018::score_subsystem::ScoreGoal::PREP_SCORE_MID_BACK, true);
  SetInput(0, false, 0, false);

  for (int i = 0; i < 4000; i++) {
    SetLoopInput(elevator_plant_.y(0), wrist_plant_.y(0));
    Update();
  }

  EXPECT_NEAR(score_subsystem_status_proto_->elevator_actual_height(),
              c2018::score_subsystem::kElevatorScoreMid, 1e-3);
  EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(),
              c2018::score_subsystem::kWristBackwardAngle, 1e-3);
  EXPECT_NEAR(score_subsystem_output_proto_->intake_voltage(), 0, 1e-3);
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(score_subsystem_output_proto_->wrist_solenoid_close());
}

TEST_F(ScoreSubsystemTest, PrepHighScoreBack) {
  SetGoal(c2018::score_subsystem::ScoreGoal::PREP_SCORE_HIGH_BACK, true);
  SetInput(0, false, 0, false);

  for (int i = 0; i < 4000; i++) {
    SetLoopInput(elevator_plant_.y(0), wrist_plant_.y(0));
    Update();
  }

  EXPECT_NEAR(score_subsystem_status_proto_->elevator_actual_height(),
              c2018::score_subsystem::kElevatorScoreHigh, 1e-3);
  EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(),
              c2018::score_subsystem::kWristBackwardAngle, 1e-3);
  EXPECT_NEAR(score_subsystem_output_proto_->intake_voltage(), 0, 1e-3);
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(score_subsystem_output_proto_->wrist_solenoid_close());
}

TEST_F(ScoreSubsystemTest, MidScoreBack) {
  SetGoal(c2018::score_subsystem::ScoreGoal::SCORE_MID_BACK, true);
  SetInput(0, false, 0, false);

  for (int i = 0; i < 4000; i++) {
    SetLoopInput(elevator_plant_.y(0), wrist_plant_.y(0));
    Update();
  }

  EXPECT_NEAR(score_subsystem_status_proto_->elevator_actual_height(),
              c2018::score_subsystem::kElevatorScoreMid, 1e-3);
  EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(),
              c2018::score_subsystem::kWristBackwardAngle, 1e-3);
  EXPECT_NEAR(score_subsystem_output_proto_->intake_voltage(), -12, 1e-3);
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_open());
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_close());
}

TEST_F(ScoreSubsystemTest, HighScoreBack) {
  SetGoal(c2018::score_subsystem::ScoreGoal::SCORE_HIGH_BACK, true);
  SetInput(0, false, 0, false);

  for (int i = 0; i < 4000; i++) {
    SetLoopInput(elevator_plant_.y(0), wrist_plant_.y(0));
    Update();
  }

  EXPECT_NEAR(score_subsystem_status_proto_->elevator_actual_height(),
              c2018::score_subsystem::kElevatorScoreHigh, 1e-3);
  EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(),
              c2018::score_subsystem::kWristBackwardAngle, 1e-3);
  EXPECT_NEAR(score_subsystem_output_proto_->intake_voltage(), -12, 1e-3);
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_open());
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_close());
}

TEST_F(ScoreSubsystemTest, IdleBottom) {
  SetGoal(c2018::score_subsystem::ScoreGoal::IDLE_BOTTOM, true);
  SetInput(0, false, 0, false);

  for (int i = 0; i < 4000; i++) {
    SetLoopInput(elevator_plant_.y(0), wrist_plant_.y(0));
    Update();
  }

  EXPECT_NEAR(score_subsystem_status_proto_->elevator_actual_height(), 0, 1e-2);
  EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(), 0, 1e-3);
  EXPECT_NEAR(score_subsystem_output_proto_->intake_voltage(), 0, 1e-3);
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(score_subsystem_output_proto_->wrist_solenoid_close());
}

TEST_F(ScoreSubsystemTest, IdleStow) {
  SetGoal(c2018::score_subsystem::ScoreGoal::IDLE_STOW, true);
  SetInput(0, false, 0, false);

  for (int i = 0; i < 4000; i++) {
    SetLoopInput(elevator_plant_.y(0), wrist_plant_.y(0));
    Update();
  }

  EXPECT_NEAR(score_subsystem_status_proto_->elevator_actual_height(), 0, 1e-2);
  EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(),
              c2018::score_subsystem::kWristStowAngle, 1e-3);
  EXPECT_NEAR(score_subsystem_output_proto_->intake_voltage(), 0, 1e-3);
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(score_subsystem_output_proto_->wrist_solenoid_close());
}

TEST_F(ScoreSubsystemTest, IntakeManual) {
  SetGoal(c2018::score_subsystem::ScoreGoal::HEIGHT_0, true);
  SetInput(0, false, 0, false);

  for (int i = 0; i < 4000; i++) {
    SetLoopInput(elevator_plant_.y(0), wrist_plant_.y(0));
    Update();
  }

  SetGoal(c2018::score_subsystem::ScoreGoal::INTAKE_MANUAL, true);
  SetLoopInput(elevator_plant_.y(0), wrist_plant_.y(0));
  Update();

  EXPECT_NEAR(score_subsystem_status_proto_->elevator_actual_height(), 0, 1e-3);
  EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(), 0, 1e-3);
  EXPECT_NEAR(score_subsystem_output_proto_->intake_voltage(), 12, 1e-3);
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_open());
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_close());
}

TEST_F(ScoreSubsystemTest, OuttakeManual) {
  SetGoal(c2018::score_subsystem::ScoreGoal::HEIGHT_0, true);
  SetInput(0, false, 0, false);

  for (int i = 0; i < 4000; i++) {
    SetLoopInput(elevator_plant_.y(0), wrist_plant_.y(0));
    Update();
  }

  SetGoal(c2018::score_subsystem::ScoreGoal::OUTTAKE_MANUAL, true);
  SetLoopInput(elevator_plant_.y(0), wrist_plant_.y(0));
  Update();

  EXPECT_NEAR(score_subsystem_status_proto_->elevator_actual_height(), 0, 1e-3);
  EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(), 0, 1e-3);
  EXPECT_NEAR(score_subsystem_output_proto_->intake_voltage(), -12, 1e-3);
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_open());
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_close());
}

TEST_F(ScoreSubsystemTest, IdleManual) {
  SetGoal(c2018::score_subsystem::ScoreGoal::HEIGHT_0, true);
  SetInput(0, false, 0, false);

  for (int i = 0; i < 4000; i++) {
    SetLoopInput(elevator_plant_.y(0), wrist_plant_.y(0));
    Update();
  }

  SetGoal(c2018::score_subsystem::ScoreGoal::IDLE_MANUAL, true);
  SetLoopInput(elevator_plant_.y(0), wrist_plant_.y(0));
  Update();

  EXPECT_NEAR(score_subsystem_status_proto_->elevator_actual_height(), 0, 1e-3);
  EXPECT_NEAR(score_subsystem_status_proto_->wrist_angle(), 0, 1e-3);
  EXPECT_NEAR(score_subsystem_output_proto_->intake_voltage(), 0, 1e-3);
  EXPECT_FALSE(score_subsystem_output_proto_->wrist_solenoid_open());
  EXPECT_TRUE(score_subsystem_output_proto_->wrist_solenoid_close());
}
