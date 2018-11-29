#include "o2018/subsystems/arm/arm.h"
#include "gtest/gtest.h"

namespace o2018 {
namespace subsystems {
namespace arm {

class ArmTest : public ::testing::Test {
 public:
  void Update() {
    arm_input_proto_->set_intake_proxy(has_cube_);
    arm_input_proto_->set_arm_voltage(voltage_);
    if (arm_output_proto_->arm_output_type() == POSITION) {
      arm_input_proto_->set_arm_encoder(arm_output_proto_->arm_setpoint() +
                                        offset_);
      arm_input_proto_->set_arm_velocity(
          (arm_output_proto_->arm_setpoint() - prev_position_) / 0.01);
      prev_position_ = arm_input_proto_->arm_encoder();
    }
    arm_input_proto_->set_arm_hall(
        std::abs(arm_status_proto_->arm_angle()) < 2e-2);
    WriteMessages();
    arm_.Update();
    ReadMessages();
  }

  bool NotBroken() {
    return (arm_status_proto_->arm_angle() > kMinAngle - 1e-6 &&
            arm_status_proto_->arm_angle() < kMaxAngle + 1e-6);
  }

  void CheckGoal() {
    EXPECT_NEAR(arm_status_proto_->arm_angle(),
                arm_status_proto_->arm_profiled_goal(), 1e-9);
    EXPECT_NEAR(arm_status_proto_->arm_angle(),
                arm_status_proto_->arm_unprofiled_goal(), 1e-9);
    switch (intake_goal_) {
      case INTAKE_NONE:
        EXPECT_TRUE(arm_output_proto_->intake_close());
        EXPECT_FALSE(arm_output_proto_->intake_open());
        if (arm_status_proto_->has_cube()) {
          EXPECT_NEAR(arm_output_proto_->intake_voltage(), kHoldingVoltage,
                      1e-9);
        } else {
          EXPECT_NEAR(arm_output_proto_->intake_voltage(), 0., 1e-9);
        }
        break;
      case INTAKE:
        EXPECT_NEAR(arm_output_proto_->intake_voltage(), kIntakeVoltage, 1e-9);
        break;
      case INTAKE_OPEN:
        EXPECT_TRUE(arm_output_proto_->intake_open());
        EXPECT_FALSE(arm_output_proto_->intake_close());
        EXPECT_NEAR(arm_output_proto_->intake_voltage(), kIntakeVoltage, 1e-9);
        break;
      case INTAKE_CLOSE:
      case SETTLE:
        EXPECT_TRUE(arm_output_proto_->intake_close());
        EXPECT_FALSE(arm_output_proto_->intake_open());
        EXPECT_NEAR(arm_output_proto_->intake_voltage(), kIntakeVoltage, 1e-9);
        break;
      case OUTTAKE_SLOW:
        EXPECT_FALSE(arm_output_proto_->intake_open());
        EXPECT_TRUE(arm_output_proto_->intake_close());
        EXPECT_NEAR(arm_output_proto_->intake_voltage(), kSlowOuttakeVoltage,
                    1e-9);
        break;
      case OUTTAKE_FAST:
        EXPECT_FALSE(arm_output_proto_->intake_open());
        EXPECT_TRUE(arm_output_proto_->intake_close());
        EXPECT_NEAR(arm_output_proto_->intake_voltage(), kFastOuttakeVoltage,
                    1e-9);
        break;
      case DROP:
        EXPECT_TRUE(arm_output_proto_->intake_open());
        EXPECT_FALSE(arm_output_proto_->intake_close());
        EXPECT_NEAR(arm_output_proto_->intake_voltage(), 0., 1e-9);
        break;
    }
  }

  void WriteMessages() {
    ds_proto_->set_is_sys_active(outputs_enabled_);
    arm_input_queue_->WriteMessage(arm_input_proto_);
    arm_goal_queue_->WriteMessage(arm_goal_proto_);
    ds_queue_->WriteMessage(ds_proto_);
  }

  void ReadMessages() {
    arm_status_reader_.ReadLastMessage(&arm_status_proto_);
    arm_output_reader_.ReadLastMessage(&arm_output_proto_);
  }

  void RunFor(int ticks) {
    for (int i = 0; i < ticks; i++) {
      Update();
      EXPECT_TRUE(NotBroken());
    }
    CheckGoal();
  }

  void SetGoal(double goal, IntakeMode intake_goal = INTAKE_NONE) {
    intake_goal_ = intake_goal;
    arm_goal_proto_->set_arm_angle(goal);
    arm_goal_proto_->set_intake_mode(intake_goal);
  }

  void SetInput(double position, bool hall) {
    arm_input_proto_->set_arm_encoder(position);
    arm_input_proto_->set_arm_hall(hall);
  }

  void CalibrateDisabled(double offset = 0) {
    offset_ = offset;
    outputs_enabled_ = false;
    calibrate_test_ = true;

    arm_input_proto_->set_arm_encoder(offset);
    arm_input_proto_->set_zeroed(true);
    Update();

    EXPECT_TRUE(arm_status_proto_->arm_calibrated());
    EXPECT_NEAR(arm_status_proto_->arm_angle(), offset, 1e-3);
  }

  ArmInputProto arm_input_proto_;
  ArmStatusProto arm_status_proto_;
  ArmOutputProto arm_output_proto_;
  ArmGoalProto arm_goal_proto_;
  muan::wpilib::DriverStationProto ds_proto_;

  bool outputs_enabled_;
  bool has_cube_ = false;
  double voltage_ = kEncoderFaultMinVoltage - 0.1;

 protected:
  Arm arm_;

 private:
  ArmStatusQueue::QueueReader arm_status_reader_{
      muan::queues::QueueManager<ArmStatusProto>::Fetch()->MakeReader()};
  ArmOutputQueue::QueueReader arm_output_reader_{
      muan::queues::QueueManager<ArmOutputProto>::Fetch()->MakeReader()};
  ArmInputQueue* arm_input_queue_{
      muan::queues::QueueManager<ArmInputProto>::Fetch()};
  ArmGoalQueue* arm_goal_queue_{
      muan::queues::QueueManager<ArmGoalProto>::Fetch()};
  muan::wpilib::DriverStationQueue* ds_queue_{
      muan::queues::QueueManager<muan::wpilib::DriverStationProto>::Fetch()};

  double prev_position_;
  double offset_;

  bool calibrate_test_;
  IntakeMode intake_goal_;
};

TEST_F(ArmTest, NotEnabled) {
  SetGoal(1);

  outputs_enabled_ = false;

  Update();

  EXPECT_EQ(arm_status_proto_->arm_angle(), 0.);
  EXPECT_EQ(arm_output_proto_->arm_output_type(), OPEN_LOOP);
  EXPECT_NEAR(arm_output_proto_->arm_setpoint(), 0., 1e-3);
}

TEST_F(ArmTest, CalibrateDisabled) { CalibrateDisabled(1.); }

TEST_F(ArmTest, AllAngles) {
  CalibrateDisabled();

  arm_input_proto_->set_arm_encoder(0);
  arm_input_proto_->set_arm_hall(false);
  outputs_enabled_ = true;

  SetGoal(0.6);
  RunFor(1000);
  CheckGoal();

  SetGoal(kMaxAngle);
  RunFor(1000);
  CheckGoal();

  SetGoal(0);
  RunFor(1000);
  CheckGoal();

  SetGoal(0.3);
  RunFor(1000);
  CheckGoal();
}

TEST_F(ArmTest, Cap) {
  CalibrateDisabled();

  arm_input_proto_->set_arm_encoder(0);
  arm_input_proto_->set_arm_hall(false);
  outputs_enabled_ = true;

  SetGoal(4000);
  RunFor(1000);
  CheckGoal();

  SetGoal(-4000);
  RunFor(1000);
  CheckGoal();
}

TEST_F(ArmTest, IntakeModes) {
  arm_input_proto_->set_arm_encoder(0);
  arm_input_proto_->set_arm_hall(false);
  outputs_enabled_ = true;
  SetGoal(0.0, INTAKE);
  Update();
  CheckGoal();

  SetGoal(0.0, INTAKE_OPEN);
  Update();
  CheckGoal();

  SetGoal(0.0, INTAKE_CLOSE);
  Update();
  CheckGoal();

  SetGoal(0.0, SETTLE);
  Update();
  CheckGoal();

  SetGoal(0.0, OUTTAKE_SLOW);
  Update();
  CheckGoal();

  SetGoal(0.0, OUTTAKE_FAST);
  Update();
  CheckGoal();

  SetGoal(0.0, DROP);
  Update();
  CheckGoal();

  SetGoal(0.0, INTAKE_NONE);
  has_cube_ = true;

  RunFor(600);
  EXPECT_TRUE(arm_status_proto_->has_cube());
  CheckGoal();
}

}  // namespace arm
}  // namespace subsystems
}  // namespace o2018
