#include "third_party/frc971/control_loops/drivetrain/drivetrain_simulation.h"
#include <stdlib.h>
#include "muan/queues/queue_manager.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {
namespace testing {

using ::aos::time::Time;
using frc971::control_loops::drivetrain::DrivetrainConfig;
using frc971::control_loops::drivetrain::GoalProto;
using frc971::control_loops::drivetrain::InputProto;
using frc971::control_loops::drivetrain::OutputProto;
using frc971::control_loops::drivetrain::StatusProto;
using muan::queues::QueueManager;
using muan::wpilib::DriverStationProto;
using muan::wpilib::gyro::GyroMessageProto;

void DrivetrainSimulation::Reinitialize() {
  drivetrain_plant_->mutable_X(0, 0) = 0.0;
  drivetrain_plant_->mutable_X(1, 0) = 0.0;
  drivetrain_plant_->mutable_Y() =
      drivetrain_plant_->C() * drivetrain_plant_->X();
  last_left_position_ = drivetrain_plant_->Y(0, 0);
  last_right_position_ = drivetrain_plant_->Y(1, 0);
}

void DrivetrainSimulation::SendPositionMessage() {
  const double left_encoder = last_left_position_;
  const double right_encoder = last_right_position_;

  {
    InputProto position;
    position->set_left_encoder(left_encoder);
    position->set_right_encoder(right_encoder);
    input_queue_->WriteMessage(position);
  }

  {
    GyroMessageProto gyro;
    gyro->set_current_angle((right_encoder - left_encoder) /
                            (drivetrain_config_.robot_radius * 2.0));
    gyro->set_current_angular_velocity(
        (drivetrain_plant_->X(3, 0) - drivetrain_plant_->X(1, 0)) /
        (drivetrain_config_.robot_radius * 2.0));
    gyro_queue_->WriteMessage(gyro);
  }
}

void DrivetrainSimulation::Simulate() {
  last_left_position_ = drivetrain_plant_->Y(0, 0);
  last_right_position_ = drivetrain_plant_->Y(1, 0);
  auto output = output_queue_.ReadLastMessage();
  ::Eigen::Matrix<double, 2, 1> U;
  if (output) {
    U << (*output)->left_voltage(), (*output)->right_voltage();
    left_gear_high_ = (*output)->high_gear();
    right_gear_high_ = (*output)->high_gear();
  }
  {
    const double scalar = 1.0;  // ::aos::robot_state->voltage_battery / 12.0;
    U *= scalar;
  }

  if (left_gear_high_) {
    if (right_gear_high_) {
      drivetrain_plant_->set_index(3);
    } else {
      drivetrain_plant_->set_index(2);
    }
  } else {
    if (right_gear_high_) {
      drivetrain_plant_->set_index(1);
    } else {
      drivetrain_plant_->set_index(0);
    }
  }

  U(0, 0) += drivetrain_plant_->left_voltage_offset();
  U(1, 0) += drivetrain_plant_->right_voltage_offset();

  // Simulate noise
  last_left_position_ += ((double)rand() / (double)RAND_MAX - 0.5) * kSensorNoise;
  last_right_position_ += ((double)rand() / (double)RAND_MAX - 0.5) * kSensorNoise;
  drivetrain_plant_->mutable_X(1, 0) *= (1 - 0.05 * (1 - kVelocityScale));
  drivetrain_plant_->mutable_X(3, 0) *= (1 - 0.05 * (1 - kVelocityScale));
  drivetrain_plant_->mutable_X(0, 0) += ((double)rand() / (double)RAND_MAX - 0.5) * kPositionNoise;
  drivetrain_plant_->mutable_X(1, 0) += ((double)rand() / (double)RAND_MAX - 0.5) * kVelocityNoise;
  drivetrain_plant_->mutable_X(2, 0) += ((double)rand() / (double)RAND_MAX - 0.5) * kPositionNoise;
  drivetrain_plant_->mutable_X(3, 0) += ((double)rand() / (double)RAND_MAX - 0.5) * kVelocityNoise;
  U(0, 0) *= kVoltageScale;
  U(1, 0) *= kVoltageScale;
  U(0, 0) += (last_U_(0, 0) - U(0, 0)) * ::std::exp(-0.05 / kVoltageLag);
  U(1, 0) += (last_U_(1, 0) - U(1, 0)) * ::std::exp(-0.05 / kVoltageLag);

  drivetrain_plant_->Update(U);
  last_U_ = U;
}

void SimulationRunner::StartMocktime()  {
  current_time_ = ::aos::monotonic_clock::epoch();
  ::aos::time::EnableMockTime(current_time_);
  ::aos::time::SetMockTime(current_time_);
}

 SimulationRunner::SimulationRunner(DrivetrainConfig drivetrain_config,
                                StateFeedbackPlant<4, 2, 2>&& drivetrain_plant)
      : goal_queue_(QueueManager<GoalProto>::Fetch()),
        input_queue_(QueueManager<InputProto>::Fetch()),
        status_queue_(QueueManager<StatusProto>::Fetch()),
        output_queue_(QueueManager<OutputProto>::Fetch()),
        gyro_queue_(QueueManager<GyroMessageProto>::Fetch()),
        driver_station_queue_(QueueManager<DriverStationProto>::Fetch()),
        drivetrain_motor_(drivetrain_config, goal_queue_, input_queue_,
                          output_queue_, status_queue_,
                          driver_station_queue_, gyro_queue_),
        drivetrain_motor_plant_(input_queue_, output_queue_, gyro_queue_,
                                ::std::move(drivetrain_plant), drivetrain_config) {
  muan::queues::ResetAllQueues();
}

void SimulationRunner::BeginLogging(std::string dirname) {
  system(("mkdir -p " + dirname).c_str());
  system(("rm " + dirname + "/*.csv").c_str());
  logger_ = std::make_unique<muan::logging::Logger>(
          std::make_unique<muan::logging::FileWriter>(dirname));
  logger_->AddQueue("goal", goal_queue_);
  logger_->AddQueue("input", input_queue_);
  logger_->AddQueue("status", status_queue_);
  logger_->AddQueue("output", output_queue_);
  logger_->AddQueue("gyro", gyro_queue_);
  logger_->AddQueue("driver_station", driver_station_queue_);
}

void SimulationRunner::RunIteration() {
  drivetrain_motor_plant_.SendPositionMessage();
  drivetrain_motor_.Update();
  drivetrain_motor_plant_.Simulate();
  SimulateTimestep(true);
  if (logger_) {
    logger_->Update();
  }
}

void SimulationRunner::RunForTime(const aos::monotonic_clock::duration run_for) {
  ::aos::time::SetMockTime(current_time_);
  const auto end_time = aos::monotonic_clock::now() + run_for;
  while (::aos::monotonic_clock::now() < end_time) {
    RunIteration();
  }
}

void SimulationRunner::SimulateTimestep(bool enabled) {
  {
    DriverStationProto ds_state;
    auto mode = enabled ? enable_mode_ : RobotMode::DISABLED;
    ds_state->set_mode(mode);
    ds_state->set_is_sys_active(mode != RobotMode::DISABLED);
    ds_state->set_brownout(false);
    ds_state->set_battery_voltage(12.0);
    ds_state->set_has_ds_connection(true);
    driver_station_queue_->WriteMessage(ds_state);
  }
  constexpr auto kTimeTick = ::std::chrono::milliseconds(5);
  ::aos::time::SetMockTime(current_time_ += kTimeTick);
}

void DrivetrainTest::VerifyNearGoal() {
  auto goal = goal_queue_->ReadLastMessage();
  auto position = input_queue_->ReadLastMessage();

  EXPECT_TRUE(goal);
  EXPECT_TRUE((*goal)->has_distance_command());
  EXPECT_TRUE(position);

  EXPECT_NEAR((*goal)->distance_command().left_goal(),
              drivetrain_motor_plant_.GetLeftPosition(), 5e-3);
  EXPECT_NEAR((*goal)->distance_command().right_goal(),
              drivetrain_motor_plant_.GetRightPosition(), 5e-3);
}

// Change these values to simulate a really awful drivetrain or one inconsistent
// with the model. Theoretical values are currently uncommented,
// sample "bad" values are in comments.
double kSensorNoise = 0;  // 0.001;
double kPositionNoise = 0;  // 0.001;
double kVelocityNoise = 0;  // 0.001;
double kVoltageScale = 1;  // 0.8;
double kVelocityScale = 1;  // 1.1;
double kVoltageLag = 0;  // 0.5;

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971
