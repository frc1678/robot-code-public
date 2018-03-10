#ifndef THIRD_PARTY_FRC971_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_SIMULATION_H_
#define THIRD_PARTY_FRC971_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_SIMULATION_H_

#include <memory>
#include <string>
#include "gtest/gtest.h"
#include "third_party/frc971/control_loops/state_feedback_loop.h"
#include "third_party/frc971/control_loops/drivetrain/drivetrain.h"
#include "muan/logging/logger.h"
#include "muan/queues/queue_manager.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {
namespace testing {

// Change these values to simuate a really awful drivetrain or one inconsistent
// with the model
double kSensorNoise = 0;  // 0.001;
double kPositionNoise = 0;  // 0.001;
double kVelocityNoise = 0;  // 0.001;
double kVoltageScale = 1;  // 0.8;
double kVelocityScale = 1;  // 1.1;
double kVoltageLag = 0;  // 0.5;

class DrivetrainPlant : public StateFeedbackPlant<4, 2, 2> {
 public:
  explicit DrivetrainPlant(StateFeedbackPlant<4, 2, 2>&& other)
      : StateFeedbackPlant<4, 2, 2>(::std::move(other)) {}

  void CheckU(const Eigen::Matrix<double, 2, 1> &U) override {
    EXPECT_LE(U(0, 0), U_max(0, 0) * kVoltageScale + 0.00001 + left_voltage_offset_);
    EXPECT_GE(U(0, 0), U_min(0, 0) * kVoltageScale - 0.00001 + left_voltage_offset_);
    EXPECT_LE(U(1, 0), U_max(1, 0) * kVoltageScale + 0.00001 + right_voltage_offset_);
    EXPECT_GE(U(1, 0), U_min(1, 0) * kVoltageScale - 0.00001 + right_voltage_offset_);
  }


  double left_voltage_offset() const { return left_voltage_offset_; }
  void set_left_voltage_offset(double left_voltage_offset) {
    left_voltage_offset_ = left_voltage_offset;
  }

  double right_voltage_offset() const { return right_voltage_offset_; }
  void set_right_voltage_offset(double right_voltage_offset) {
    right_voltage_offset_ = right_voltage_offset;
  }

 private:
  double left_voltage_offset_ = 0.0;
  double right_voltage_offset_ = 0.0;
};

// Class which simulates the drivetrain and sends out queue messages containing
// the position.
class DrivetrainSimulation {
 public:
  // Constructs a motor simulation.
  // TODO(aschuh) Do we want to test the clutch one too?
  DrivetrainSimulation(
      ::frc971::control_loops::drivetrain::InputQueue* input_queue,
      ::frc971::control_loops::drivetrain::OutputQueue* output_queue,
      ::muan::wpilib::gyro::GyroQueue* gyro_queue,
      StateFeedbackPlant<4, 2, 2>&& drivetrain_plant,
      ::frc971::control_loops::drivetrain::DrivetrainConfig drivetrain_config)
      : drivetrain_plant_(::std::make_unique<DrivetrainPlant>(
              ::std::move(drivetrain_plant))),
        input_queue_(input_queue),
        output_queue_(output_queue->MakeReader()),
        gyro_queue_(gyro_queue),
        drivetrain_config_(drivetrain_config) {
    Reinitialize();
    last_U_.setZero();
  }

  // Resets the plant.
  void Reinitialize();

  // Returns the position of the drivetrain.
  double GetLeftPosition() const { return drivetrain_plant_->Y(0, 0); }
  double GetRightPosition() const { return drivetrain_plant_->Y(1, 0); }

  // Sends out the position queue messages.
  void SendPositionMessage();

  // Simulates the drivetrain moving for one timestep.
  void Simulate();

  void set_left_voltage_offset(double left_voltage_offset) {
    drivetrain_plant_->set_left_voltage_offset(left_voltage_offset);
  }
  void set_right_voltage_offset(double right_voltage_offset) {
    drivetrain_plant_->set_right_voltage_offset(right_voltage_offset);
  }

  ::std::unique_ptr<DrivetrainPlant> drivetrain_plant_;
 private:
  ::frc971::control_loops::drivetrain::InputQueue* input_queue_;
  ::frc971::control_loops::drivetrain::OutputQueue::QueueReader output_queue_;

  ::muan::wpilib::gyro::GyroQueue* gyro_queue_;

  ::frc971::control_loops::drivetrain::DrivetrainConfig drivetrain_config_;

  double last_left_position_;
  double last_right_position_;

  Eigen::Matrix<double, 2, 1> last_U_;

  bool left_gear_high_ = false;
  bool right_gear_high_ = false;
};

class DrivetrainTest : public ::testing::Test {
 public:
  DrivetrainTest(::frc971::control_loops::drivetrain::DrivetrainConfig drivetrain_config,
                 StateFeedbackPlant<4, 2, 2>&& drivetrain_plant);

  void BeginLogging(::std::string dirname);

 protected:
  ::frc971::control_loops::drivetrain::GoalQueue* goal_queue_;
  ::frc971::control_loops::drivetrain::InputQueue* input_queue_;
  ::frc971::control_loops::drivetrain::StatusQueue* status_queue_;
  ::frc971::control_loops::drivetrain::OutputQueue* output_queue_;

  ::muan::wpilib::gyro::GyroQueue* gyro_queue_;
  ::muan::wpilib::DriverStationQueue* driver_station_queue_;

  ::std::unique_ptr<muan::logging::Logger> logger_;

  // Create a loop and simulation plant.
  DrivetrainLoop drivetrain_motor_;
  DrivetrainSimulation drivetrain_motor_plant_;

  ::aos::monotonic_clock::time_point current_time_ = ::aos::monotonic_clock::epoch();

  RobotMode enable_mode_ = RobotMode::TELEOP;

  void SetUp() override;

  void RunIteration();

  void RunForTime(const aos::monotonic_clock::duration run_for);

  void SimulateTimestep(bool enabled);

  void VerifyNearGoal();

  virtual ~DrivetrainTest() {}
};

}  // namespace testing
}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // THIRD_PARTY_FRC971_CONTROL_LOOPS_DRIVETRAIN_DRIVETRAIN_SIMULATION_H_
