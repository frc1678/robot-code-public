#include "elevator_controller.h"
#include "gtest/gtest.h"

class ElevatorControllerTest : public ::testing::Test {
 public:
  ElevatorControllerTest() {
    plant_ = muan::control::StateSpacePlant<1, 3, 1>(
        frc1678::elevator_controller::controller::first_stage_integral::A(),
        frc1678::elevator_controller::controller::first_stage_integral::B(),
        frc1678::elevator_controller::controller::first_stage_integral::C());
  }

  void Update() {
    if (plant_.x(0) < 0) {
      plant_.x(0) = 0;
    }
    elevator_input_proto_->set_elevator_hall(plant_.x(0) >= 0.04 && plant_.x(0) <= 0.06);
    elevator_.Update(elevator_input_proto_, &elevator_output_proto_, &elevator_status_proto_,
                     outputs_enabled_);
    SetWeights(plant_.x()(0, 0) >= 1.0, elevator_input_proto_->has_cube());
    plant_.Update((Eigen::Matrix<double, 1, 1>() << elevator_output_proto_->elevator_voltage()).finished());
  }

  void SetGoal(double goal) { elevator_.SetGoal(goal); }

  void SetInput(double position, bool hall) {
    elevator_input_proto_->set_elevator_encoder(position);
    elevator_input_proto_->set_elevator_hall(hall);
  }

  c2018::score_subsystem::ScoreSubsystemInputProto elevator_input_proto_;
  c2018::score_subsystem::ScoreSubsystemStatusProto elevator_status_proto_;
  c2018::score_subsystem::ScoreSubsystemOutputProto elevator_output_proto_;

  bool outputs_enabled_;

 protected:
  muan::control::StateSpacePlant<1, 3, 1> plant_;

 private:
  c2018::score_subsystem::elevator::ElevatorController elevator_;

  void SetWeights(bool second_stage, bool has_cube) {
    if (second_stage && has_cube) {
      plant_.A() = frc1678::elevator_controller::controller::second_stage_cube_integral::A();
      plant_.B() = frc1678::elevator_controller::controller::second_stage_cube_integral::B();
      plant_.C() = frc1678::elevator_controller::controller::second_stage_cube_integral::C();
    } else if (second_stage && !has_cube) {
      plant_.A() = frc1678::elevator_controller::controller::second_stage_integral::A();
      plant_.B() = frc1678::elevator_controller::controller::second_stage_integral::B();
      plant_.C() = frc1678::elevator_controller::controller::second_stage_integral::C();
    } else if (!second_stage && has_cube) {
      plant_.A() = frc1678::elevator_controller::controller::first_stage_cube_integral::A();
      plant_.B() = frc1678::elevator_controller::controller::first_stage_cube_integral::B();
      plant_.C() = frc1678::elevator_controller::controller::first_stage_cube_integral::C();
    } else if (!second_stage && !has_cube) {
      plant_.A() = frc1678::elevator_controller::controller::first_stage_integral::A();
      plant_.B() = frc1678::elevator_controller::controller::first_stage_integral::B();
      plant_.C() = frc1678::elevator_controller::controller::first_stage_integral::C();
    }
  }
};

TEST_F(ElevatorControllerTest, NotEnabled) {
  SetGoal(1);

  outputs_enabled_ = false;

  Update();

  EXPECT_EQ(elevator_status_proto_->elevator_actual_height(), 0.);
  EXPECT_NEAR(elevator_output_proto_->elevator_voltage(), 0., 1e-3);
}

TEST_F(ElevatorControllerTest, Calibration) {
  elevator_input_proto_->set_elevator_encoder(0);
  elevator_input_proto_->set_elevator_hall(false);
  outputs_enabled_ = true;

  double offset = 0.2;

  SetGoal(2.06);

  for (int i = 0; i < 2000; i++) {
    elevator_input_proto_->set_elevator_encoder(plant_.y(0) + offset);
    Update();
    EXPECT_TRUE(elevator_output_proto_->elevator_voltage() >=
                muan::utils::Cap(elevator_output_proto_->elevator_voltage(), -12, 12) - 0.01);
  }

  EXPECT_NEAR(elevator_status_proto_->elevator_actual_height(), 2.06, 1e-3);
  EXPECT_TRUE(elevator_status_proto_->elevator_calibrated());
}

TEST_F(ElevatorControllerTest, Heights) {
  elevator_input_proto_->set_elevator_encoder(0);
  elevator_input_proto_->set_elevator_hall(false);
  outputs_enabled_ = true;

  SetGoal(2.06);

  for (int i = 0; i < 2000; i++) {
    elevator_input_proto_->set_elevator_encoder(plant_.y(0));
    Update();
    EXPECT_TRUE(elevator_output_proto_->elevator_voltage() >=
                muan::utils::Cap(elevator_output_proto_->elevator_voltage(), -12, 12) - 0.01);
  }

  EXPECT_TRUE(elevator_status_proto_->elevator_calibrated());
  EXPECT_TRUE(elevator_status_proto_->elevator_at_top());
  EXPECT_NEAR(elevator_status_proto_->elevator_actual_height(), 2.06, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_unprofiled_goal(), 2.06, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_profiled_goal(), 2.06, 1e-3);
  SetGoal(0.6);

  for (int i = 0; i < 2000; i++) {
    elevator_input_proto_->set_elevator_encoder(plant_.y(0));
    Update();
    EXPECT_TRUE(elevator_output_proto_->elevator_voltage() >=
                muan::utils::Cap(elevator_output_proto_->elevator_voltage(), -12, 12) - 0.01);
  }

  EXPECT_NEAR(elevator_status_proto_->elevator_actual_height(), 0.6, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_unprofiled_goal(), 0.6, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_profiled_goal(), 0.6, 1e-3);
  SetGoal(0);

  for (int i = 0; i < 2000; i++) {
    elevator_input_proto_->set_elevator_encoder(plant_.y(0));
    Update();
    EXPECT_TRUE(elevator_output_proto_->elevator_voltage() >=
                muan::utils::Cap(elevator_output_proto_->elevator_voltage(), -12, 12) - 0.01);
  }

  EXPECT_NEAR(elevator_status_proto_->elevator_actual_height(), 0, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_unprofiled_goal(), 0, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_profiled_goal(), 0, 1e-3);
  SetGoal(0.3);

  for (int i = 0; i < 2000; i++) {
    elevator_input_proto_->set_elevator_encoder(plant_.y(0));
    Update();
    EXPECT_TRUE(elevator_output_proto_->elevator_voltage() >=
                muan::utils::Cap(elevator_output_proto_->elevator_voltage(), -12, 12) - 0.01);
  }

  EXPECT_NEAR(elevator_status_proto_->elevator_actual_height(), 0.3, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_unprofiled_goal(), 0.3, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_profiled_goal(), 0.3, 1e-3);
}

TEST_F(ElevatorControllerTest, EncoderFault) {
  elevator_input_proto_->set_elevator_encoder(0);
  elevator_input_proto_->set_elevator_hall(false);
  outputs_enabled_ = true;

  SetGoal(2.06);

  for (int i = 0; i < 400; i++) {
    elevator_input_proto_->set_elevator_encoder(0);
    Update();
    EXPECT_TRUE(elevator_output_proto_->elevator_voltage() >=
                muan::utils::Cap(elevator_output_proto_->elevator_voltage(), -12, 12) - 0.01);
  }

  EXPECT_TRUE(elevator_status_proto_->elevator_encoder_fault_detected());
  EXPECT_NEAR(elevator_status_proto_->elevator_actual_height(), 0, 1e-3);
}

TEST_F(ElevatorControllerTest, HeightToHigh) {
  elevator_input_proto_->set_elevator_encoder(0);
  elevator_input_proto_->set_elevator_hall(false);
  outputs_enabled_ = true;

  SetGoal(4000);

  for (int i = 0; i < 400; i++) {
    elevator_input_proto_->set_elevator_encoder(plant_.y(0));
    Update();
    EXPECT_TRUE(elevator_output_proto_->elevator_voltage() >=
                muan::utils::Cap(elevator_output_proto_->elevator_voltage(), -12, 12) - 0.01);
  }

  EXPECT_NEAR(elevator_status_proto_->elevator_actual_height(), 2.06, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_unprofiled_goal(), 2.06, 1e-3);
  EXPECT_NEAR(elevator_status_proto_->elevator_profiled_goal(), 2.06, 1e-3);
}
