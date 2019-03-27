#include "c2019/subsystems/superstructure/superstructure.h"
#include "gtest/gtest.h"
#include "muan/queues/queue_manager.h"

namespace c2019 {

namespace superstructure {

class SuperstructureTest : public ::testing::Test {
 public:
  SuperstructureTest() {}

  muan::wpilib::DriverStationProto driver_station_proto_;
  SuperstructureGoalProto superstructure_goal_proto_;
  SuperstructureInputProto superstructure_input_proto_;
  SuperstructureStatusProto superstructure_status_proto_;
  SuperstructureOutputProto superstructure_output_proto_;

  muan::wpilib::DriverStationQueue* driver_station_queue_ =
      muan::queues::QueueManager<muan::wpilib::DriverStationProto>::Fetch();

  SuperstructureInputQueue* superstructure_input_queue_ =
      muan::queues::QueueManager<SuperstructureInputProto>::Fetch();

  SuperstructureGoalQueue* superstructure_goal_queue_ =
      muan::queues::QueueManager<SuperstructureGoalProto>::Fetch();

  SuperstructureStatusQueue::QueueReader superstructure_status_queue_ =
      muan::queues::QueueManager<SuperstructureStatusProto>::Fetch()
          ->MakeReader();

  SuperstructureOutputQueue::QueueReader superstructure_output_queue_ =
      muan::queues::QueueManager<SuperstructureOutputProto>::Fetch()
          ->MakeReader();

  bool outputs_enabled_;

  void CalibrateDisabled() {
    driver_station_proto_->set_is_sys_active(false);
    superstructure_input_proto_->set_elevator_encoder(0);
    superstructure_input_proto_->set_elevator_zeroed(true);
    for (int i = 0; i < 2500; i++) {
      SetInput(0, true, i * (M_PI / 2500.));
      Update();
    }

    EXPECT_TRUE(superstructure_status_proto_->elevator_is_calibrated());
    EXPECT_TRUE(superstructure_status_proto_->wrist_is_calibrated());
    EXPECT_EQ(superstructure_status_proto_->state(),
              SuperstructureState::HOLDING);
  }

  void Update() {
    WriteMessages();
    superstructure_.Update();
    ReadMessages();
    UpdateInputs();
  }

  void RunFor(int num_ticks) {
    for (int i = 0; i < num_ticks; i++) {
      Update();
    }
    LogicCheck();
  }

  void LogicCheck() {
    if (superstructure_status_proto_->wrist_angle() > kWristSafeForwardsAngle &&
        superstructure_status_proto_->wrist_angle() <
            kWristSafeBackwardsAngle) {
      EXPECT_NEAR(superstructure_status_proto_->elevator_goal(), 0, 1e-3);
    }
  }

  void ReadMessages() {
    superstructure_output_queue_.ReadLastMessage(&superstructure_output_proto_);
    superstructure_status_queue_.ReadLastMessage(&superstructure_status_proto_);
  }

  void WriteMessages() {
    superstructure_input_queue_->WriteMessage(superstructure_input_proto_);
    superstructure_goal_queue_->WriteMessage(superstructure_goal_proto_);
    driver_station_queue_->WriteMessage(driver_station_proto_);
  }

  void SetGoal(ScoreGoal score_goal, IntakeGoal intake_goal,
               bool outputs_enabled) {
    superstructure_goal_proto_->set_score_goal(score_goal);
    superstructure_goal_proto_->set_intake_goal(intake_goal);
    driver_station_proto_->set_is_sys_active(outputs_enabled);
  }

  void UpdateInputs() {
    superstructure_input_proto_->set_elevator_encoder(
        superstructure_output_proto_->elevator_setpoint());
    superstructure_input_proto_->set_wrist_encoder(
        superstructure_output_proto_->wrist_setpoint());
  }

  bool Godmode(double elevator_god_mode_goal, double wrist_god_mode_goal,
               double run_for, bool outputs_enabled) {
    double first_elevator_goal =
        superstructure_output_proto_->elevator_setpoint();
    double first_wrist_goal = superstructure_output_proto_->wrist_setpoint();

    superstructure_goal_proto_->set_elevator_god_mode_goal(
        elevator_god_mode_goal);
    superstructure_goal_proto_->set_wrist_god_mode_goal(wrist_god_mode_goal);
    RunFor(run_for);

    SetGodmodeZero(true);
    RunFor(3);

    double second_elevator_goal =
        superstructure_output_proto_->elevator_setpoint();
    double second_wrist_goal = superstructure_output_proto_->wrist_setpoint();

    if (elevator_god_mode_goal != 0 && wrist_god_mode_goal == 0) {
      if (std::abs(second_elevator_goal - first_elevator_goal) > 2e-3) {
        return true;
      } else {
        return false;
      }
    } else if (wrist_god_mode_goal != 0 && elevator_god_mode_goal == 0) {
      if (std::abs(second_wrist_goal - first_wrist_goal) > 3 * (M_PI / 180)) {
        return true;
      } else {
        return false;
      }
    } else {
      if (std::abs(second_elevator_goal - first_elevator_goal) > 2e-3 &&
          std::abs(second_wrist_goal - first_wrist_goal) > 3 * (M_PI / 180)) {
        return true;
      } else {
        return false;
      }
    }

    driver_station_proto_->set_is_sys_active(outputs_enabled);
  }

  void SetGodmodeZero(bool outputs_enabled) {
    superstructure_goal_proto_->set_elevator_god_mode_goal(0);
    superstructure_goal_proto_->set_wrist_god_mode_goal(0);
    driver_station_proto_->set_is_sys_active(outputs_enabled);
  }

  void SetInput(double elevator_encoder, bool elevator_zeroed,
                double wrist_encoder, bool wrist_zeroed = true) {
    superstructure_input_proto_->set_elevator_encoder(elevator_encoder);
    superstructure_input_proto_->set_wrist_encoder(wrist_encoder);
    superstructure_input_proto_->set_elevator_zeroed(elevator_zeroed);
    superstructure_input_proto_->set_wrist_zeroed(wrist_zeroed);
  }

  void SetIntakeInputs(bool has_ground_hatch, bool has_hp_hatch,
                       bool has_cargo) {
    superstructure_input_proto_->set_hatch_ground_current(5e6 *
                                                          has_ground_hatch);
    superstructure_input_proto_->set_hatch_intake_proxy(has_hp_hatch);
    superstructure_input_proto_->set_cargo_proxy(has_cargo);
    WriteMessages();
  }

 protected:
  void CheckGoal(double elevator, double wrist) const {
    EXPECT_NEAR(superstructure_status_proto_->elevator_goal(), elevator, 1e-3);
    EXPECT_NEAR(superstructure_status_proto_->wrist_goal(), wrist, 1e-3);

    EXPECT_NEAR(superstructure_status_proto_->elevator_height(), elevator,
                1e-3);
    EXPECT_NEAR(superstructure_status_proto_->wrist_angle(), wrist, 1e-3);

    EXPECT_NEAR(superstructure_output_proto_->elevator_setpoint(), elevator,
                1e-3);
    EXPECT_NEAR(superstructure_output_proto_->wrist_setpoint(), wrist, 1e-3);
  }

  void CheckIntake(bool has_ground_hatch, bool has_hp_hatch, bool has_cargo,
                   bool arrow_solenoid, bool backplate_solenoid,
                   bool snap_down) const {
    EXPECT_EQ(superstructure_status_proto_->has_ground_hatch(),
              has_ground_hatch);
    EXPECT_EQ(superstructure_status_proto_->has_hp_hatch(), has_hp_hatch);
    EXPECT_EQ(has_cargo, superstructure_status_proto_->has_cargo());

    EXPECT_EQ(superstructure_output_proto_->arrow_solenoid(), arrow_solenoid);
    EXPECT_EQ(superstructure_output_proto_->backplate_solenoid(),
              backplate_solenoid);
    EXPECT_EQ(superstructure_output_proto_->snap_down(), snap_down);
  }

 private:
  Superstructure superstructure_;
};

TEST_F(SuperstructureTest, CalibrateDisabled) { CalibrateDisabled(); }

TEST_F(SuperstructureTest, Disabled) {
  SetGoal(ScoreGoal::HATCH_ROCKET_THIRD, IntakeGoal::INTAKE_NONE, false);
  SetInput(0, false, 0);

  Update();

  CheckIntake(false, false, false, true, false, false);
  EXPECT_EQ(superstructure_output_proto_->elevator_setpoint(), 0);
  EXPECT_EQ(superstructure_output_proto_->wrist_setpoint(), 0);
  EXPECT_EQ(superstructure_output_proto_->cargo_roller_voltage(), 0);
  EXPECT_EQ(superstructure_output_proto_->hatch_roller_voltage(), 0);
  EXPECT_EQ(superstructure_output_proto_->left_winch_voltage(), 0);
  EXPECT_EQ(superstructure_output_proto_->drop_forks(), false);
}

TEST_F(SuperstructureTest, ScoreGoals) {
  CalibrateDisabled();

  // CARGO_ROCKET_FIRST
  SetGoal(ScoreGoal::CARGO_ROCKET_FIRST, IntakeGoal::INTAKE_NONE, true);
  RunFor(3);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(10);
  CheckGoal(kCargoRocketFirstHeight, kCargoRocketFirstAngle);

  // CARGO_ROCKET_BACKWARDS
  /* SetGoal(ScoreGoal::CARGO_ROCKET_BACKWARDS, IntakeGoal::INTAKE_NONE, true); */
  /* RunFor(3); */
  /* SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true); */
  /* RunFor(10); */
  /* CheckGoal(kCargoRocketBackwardsHeight, kCargoRocketBackwardsAngle); */

  // CARGO_ROCKET_SECOND
  SetGoal(ScoreGoal::CARGO_ROCKET_SECOND, IntakeGoal::INTAKE_NONE, true);
  RunFor(3);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(10);
  CheckGoal(kCargoRocketSecondHeight, kCargoRocketSecondAngle);

  // CARGO_ROCKET_THIRD
  SetGoal(ScoreGoal::CARGO_ROCKET_THIRD, IntakeGoal::INTAKE_NONE, true);
  RunFor(3);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(10);
  CheckGoal(kCargoRocketThirdHeight, kCargoRocketThirdAngle);

  // HATCH_ROCKET_FIRST
  SetGoal(ScoreGoal::HATCH_ROCKET_FIRST, IntakeGoal::INTAKE_NONE, true);
  RunFor(3);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(10);
  CheckGoal(kHatchRocketFirstHeight, kHatchForwardsAngle);

  // HATCH_ROCKET_BACKWARDS
  /* SetGoal(ScoreGoal::HATCH_ROCKET_BACKWARDS, IntakeGoal::INTAKE_NONE, true); */
  /* RunFor(3); */
  /* SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true); */
  /* RunFor(10); */
  /* CheckGoal(kHatchRocketBackwardsHeight, kHatchBackwardsAngle); */

  // HATCH_ROCKET_SECOND
  SetGoal(ScoreGoal::HATCH_ROCKET_SECOND, IntakeGoal::INTAKE_NONE, true);
  RunFor(3);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(10);
  CheckGoal(kHatchRocketSecondHeight, kHatchForwardsAngle);

  // HATCH_ROCKET_THIRD
  SetGoal(ScoreGoal::HATCH_ROCKET_THIRD, IntakeGoal::INTAKE_NONE, true);
  RunFor(3);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(10);
  CheckGoal(kHatchRocketThirdHeight, kHatchForwardsAngle);

  // CARGO_SHIP_FORWARDS
  SetGoal(ScoreGoal::CARGO_SHIP_FORWARDS, IntakeGoal::INTAKE_NONE, true);
  RunFor(3);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(10);
  CheckGoal(kCargoShipForwardsHeight, kCargoShipForwardsAngle);

  // CARGO_SHIP_BACKWARDS
  /* SetGoal(ScoreGoal::CARGO_SHIP_BACKWARDS, IntakeGoal::INTAKE_NONE, true); */
  /* RunFor(3); */
  /* SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true); */
  /* RunFor(10); */
  /* CheckGoal(kCargoShipBackwardsHeight, kCargoShipBackwardsAngle); */

  // HATCH_SHIP_FORWARDS
  SetGoal(ScoreGoal::HATCH_SHIP_FORWARDS, IntakeGoal::INTAKE_NONE, true);
  RunFor(3);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(10);
  CheckGoal(kHatchShipForwardsHeight, kHatchForwardsAngle);

  // HATCH_SHIP_BACKWARDS
  /* SetGoal(ScoreGoal::HATCH_SHIP_BACKWARDS, IntakeGoal::INTAKE_NONE, true); */
  /* RunFor(3); */
  /* SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true); */
  /* RunFor(10); */
  /* CheckGoal(kHatchShipBackwardsHeight, kHatchBackwardsAngle); */

  // STOW
  SetGoal(ScoreGoal::STOW, IntakeGoal::INTAKE_NONE, true);
  RunFor(3);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(10);
}

TEST_F(SuperstructureTest, IntakeGoals) {
  CalibrateDisabled();

  SetIntakeInputs(false, false, false);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);

  // INTAKE_HATCH
  SetIntakeInputs(false, false, false);

  SetGoal(ScoreGoal::HATCH_SHIP_FORWARDS, IntakeGoal::INTAKE_HATCH, true);
  RunFor(2);
  EXPECT_EQ(superstructure_status_proto_->state(), INTAKING_WRIST);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(2);
  CheckGoal(kHatchShipForwardsHeight, kHatchForwardsAngle);
  EXPECT_EQ(superstructure_status_proto_->state(), INTAKING_WRIST);

  SetIntakeInputs(false, true, false);
  RunFor(2);
  EXPECT_EQ(superstructure_status_proto_->state(), HOLDING);
  EXPECT_TRUE(superstructure_status_proto_->has_hp_hatch());

  // INTAKE_CARGO
  // Get Rid of Hatch First
  SetIntakeInputs(false, false, false);

  SetGoal(ScoreGoal::CARGO_GROUND, IntakeGoal::INTAKE_CARGO, true);
  RunFor(1);
  EXPECT_EQ(superstructure_status_proto_->state(), 4);

  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(2);
  SetIntakeInputs(false, false, true);
  RunFor(20);
  EXPECT_TRUE(superstructure_status_proto_->has_cargo());
  EXPECT_EQ(superstructure_status_proto_->state(), HOLDING);
  EXPECT_EQ(superstructure_output_proto_->cargo_roller_voltage(), 4);

  SetGoal(ScoreGoal::NONE, IntakeGoal::OUTTAKE_CARGO, true);
  RunFor(2);
  EXPECT_EQ(superstructure_output_proto_->cargo_roller_voltage(), -10);

  SetIntakeInputs(false, false, false);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(10);
  EXPECT_EQ(superstructure_output_proto_->cargo_roller_voltage(), 0);

  // INTAKE_GROUND
  // Get Rid of Cargo
  SetIntakeInputs(false, false, false);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_GROUND_HATCH, true);
  RunFor(2);
  EXPECT_EQ(superstructure_status_proto_->state(), INTAKING_GROUND);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(2);
  EXPECT_EQ(superstructure_status_proto_->state(), INTAKING_GROUND);
  EXPECT_EQ(superstructure_output_proto_->hatch_roller_voltage(), 12);
  SetIntakeInputs(true, false, false);
  RunFor(10);
  EXPECT_EQ(superstructure_status_proto_->state(), HOLDING);
  EXPECT_EQ(superstructure_output_proto_->hatch_roller_voltage(),
            ground_hatch_intake::kHoldingVoltage);

  // Spit the ground hatch to get rid of everything
  SetIntakeInputs(false, false, false);
  SetGoal(ScoreGoal::NONE, IntakeGoal::OUTTAKE_GROUND_HATCH, true);
  RunFor(100);
  EXPECT_EQ(superstructure_status_proto_->state(), HOLDING);
  EXPECT_FALSE(superstructure_status_proto_->has_ground_hatch());
  EXPECT_FALSE(superstructure_status_proto_->has_hp_hatch());

  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(3);
  SetGoal(ScoreGoal::NONE, IntakeGoal::OUTTAKE_HATCH, true);
  RunFor(1);

  EXPECT_TRUE(superstructure_output_proto_->backplate_solenoid());
  EXPECT_FALSE(superstructure_output_proto_->arrow_solenoid());
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(200);
  EXPECT_FALSE(superstructure_output_proto_->backplate_solenoid());
  EXPECT_TRUE(superstructure_output_proto_->arrow_solenoid());
}

TEST_F(SuperstructureTest, Climb) {
  CalibrateDisabled();
  SetGoal(ScoreGoal::CLIMB, IntakeGoal::INTAKE_NONE, true);
  RunFor(3);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);

  CheckGoal(kClimbHeight, kClimbAngle);
  EXPECT_TRUE(superstructure_output_proto_->elevator_high_gear());
}

TEST_F(SuperstructureTest, BuddyClimb) {
  CalibrateDisabled();
  SetGoal(ScoreGoal::DROP_FORKS, IntakeGoal::INTAKE_NONE, true);
  RunFor(3);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  EXPECT_TRUE(superstructure_output_proto_->drop_forks());

  SetGoal(ScoreGoal::DROP_CRAWLERS, IntakeGoal::INTAKE_NONE, true);
  RunFor(3);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);
  EXPECT_TRUE(superstructure_output_proto_->crawler_one_solenoid());
  EXPECT_TRUE(superstructure_output_proto_->crawler_two_solenoid());

  SetGoal(ScoreGoal::WINCH, IntakeGoal::INTAKE_NONE, true);
  RunFor(3);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);

  EXPECT_EQ(superstructure_output_proto_->left_winch_voltage(), 0);

  SetGoal(ScoreGoal::CLIMB, IntakeGoal::INTAKE_NONE, true);
  RunFor(3);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);

  SetGoal(ScoreGoal::CRAWL, IntakeGoal::INTAKE_NONE, true);
  RunFor(3);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);

  CheckGoal(kClimbHeight, kClimbAngle);
  EXPECT_EQ(superstructure_output_proto_->crawler_voltage(), 12);
  EXPECT_TRUE(superstructure_output_proto_->crawler_one_solenoid());
  EXPECT_TRUE(superstructure_output_proto_->crawler_two_solenoid());
  EXPECT_FALSE(superstructure_output_proto_->elevator_high_gear());
}

TEST_F(SuperstructureTest, Crawl) {
  CalibrateDisabled();

  SetGoal(ScoreGoal::CLIMB, IntakeGoal::INTAKE_NONE, true);
  RunFor(3);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);

  CheckGoal(kClimbHeight, kClimbAngle);
  EXPECT_TRUE(superstructure_output_proto_->elevator_high_gear());

  SetGoal(ScoreGoal::CRAWL, IntakeGoal::INTAKE_NONE, true);
  RunFor(3);
  SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true);
  RunFor(1000);

  CheckGoal(kClimbHeight, kClimbAngle);
  EXPECT_TRUE(superstructure_output_proto_->elevator_high_gear());
  EXPECT_EQ(superstructure_output_proto_->crawler_voltage(), 12);
  EXPECT_TRUE(superstructure_output_proto_->crawler_one_solenoid());
  EXPECT_TRUE(superstructure_output_proto_->crawler_two_solenoid());
}

/* TEST_F(SuperstructureTest, CrawlBraked) { */
/*   CalibrateDisabled(); */

/*   SetGoal(ScoreGoal::CLIMB, IntakeGoal::INTAKE_NONE, true); */
/*   RunFor(3); */
/*   SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true); */
/*   RunFor(1000); */

/*   CheckGoal(kClimbHeight, kClimbAngle); */
/*   EXPECT_FALSE(superstructure_output_proto_->elevator_high_gear()); */

/*   SetGoal(ScoreGoal::CRAWL_BRAKED, IntakeGoal::INTAKE_NONE, true); */
/*   RunFor(3); */
/*   SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true); */
/*   RunFor(1000); */

/*   CheckGoal(kClimbHeight, kClimbAngle); */
/*   EXPECT_FALSE(superstructure_output_proto_->elevator_high_gear()); */
/*   EXPECT_TRUE(superstructure_output_proto_->crawler_one_solenoid()); */
/*   EXPECT_TRUE(superstructure_output_proto_->crawler_two_solenoid()); */
/*   EXPECT_EQ(superstructure_output_proto_->crawler_voltage(), 12); */
/*   EXPECT_TRUE(superstructure_output_proto_->brake()); */
/* } */

/* TEST_F(SuperstructureTest, Brake) { */
/*   CalibrateDisabled(); */

/*   SetGoal(ScoreGoal::HATCH_ROCKET_THIRD, IntakeGoal::INTAKE_NONE, true); */
/*   RunFor(3); */
/*   SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true); */
/*   RunFor(1000); */

/*   CheckGoal(kHatchRocketThirdHeight, kHatchForwardsAngle); */

/*   aos::time::EnableMockTime(aos::monotonic_clock::now()); */

/*   SetGoal(ScoreGoal::BRAKE, IntakeGoal::INTAKE_NONE, true); */
/*   RunFor(3); */
/*   SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true); */
/*   aos::time::IncrementMockTime(std::chrono::milliseconds(210)); */

/*   RunFor(1000); */

/*   EXPECT_EQ(superstructure_output_proto_->elevator_setpoint(), 0); */
/*   EXPECT_EQ(superstructure_output_proto_->elevator_setpoint_type(), OPEN_LOOP); */
/*   EXPECT_TRUE(superstructure_output_proto_->brake()); */
/*   EXPECT_TRUE(superstructure_status_proto_->braked()); */
/* } */

/* TEST_F(SuperstructureTest, Handoff) { */
/*   CalibrateDisabled(); */
/*   SetIntakeInputs(false, false, false); */
/*   EXPECT_FALSE(superstructure_status_proto_->has_hp_hatch()); */
/*   EXPECT_FALSE(superstructure_status_proto_->has_ground_hatch()); */

/*   SetGoal(ScoreGoal::HANDOFF, IntakeGoal::INTAKE_GROUND_HATCH, true); */
/*   RunFor(3); */
/*   SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true); */
/*   RunFor(1000); */

/*   CheckGoal(kHandoffHeight, kHandoffAngle); */

/*   SetIntakeInputs(true, false, false); */
/*   SetGoal(ScoreGoal::NONE, IntakeGoal::POP, true); */
/*   RunFor(3); */
/*   SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true); */
/*   RunFor(1000); */

/*   SetIntakeInputs(true, true, false); */
/*   SetGoal(ScoreGoal::STOW, IntakeGoal::INTAKE_NONE, true); */
/*   RunFor(3); */
/*   SetGoal(ScoreGoal::NONE, IntakeGoal::INTAKE_NONE, true); */
/*   RunFor(1000); */

/*   SetIntakeInputs(false, true, false); */

/*   EXPECT_EQ(superstructure_status_proto_->state(), HOLDING); */
/* } */

}  // namespace superstructure
}  // namespace c2019
