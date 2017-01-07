#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_H_

#include "Eigen/Dense"

#include "muan/wpilib/gyro/queue_types.h"
#include "muan/wpilib/queue_types.h"

#include "third_party/aos/common/controls/polytope.h"
#include "third_party/aos/common/controls/polytope.h"
#include "third_party/frc971/control_loops/drivetrain/drivetrain_config.h"
#include "third_party/frc971/control_loops/drivetrain/polydrivetrain.h"
#include "third_party/frc971/control_loops/drivetrain/queue_types.h"
#include "third_party/frc971/control_loops/drivetrain/ssdrivetrain.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

class DrivetrainLoop {
 public:
  // Constructs a control loop which can take a Drivetrain or defaults to the
  // drivetrain at frc971::control_loops::drivetrain
  explicit DrivetrainLoop(
      const DrivetrainConfig& dt_config,
      ::frc971::control_loops::drivetrain::GoalQueue* goal_queue,
      ::frc971::control_loops::drivetrain::InputQueue* input_queue,
      ::frc971::control_loops::drivetrain::OutputQueue* output_queue,
      ::frc971::control_loops::drivetrain::StatusQueue* status_queue,
      ::muan::wpilib::DriverStationQueue* driver_station_queue,
      ::muan::wpilib::gyro::GyroQueue* gyro_queue);

  int ControllerIndexFromGears();

  // Executes one cycle, pulling messages from the relevant queues
  void Update();

 protected:
  // Executes one cycle of the control loop.
  void RunIteration(
      const ::frc971::control_loops::drivetrain::GoalProto* goal,
      const ::frc971::control_loops::drivetrain::InputProto* input,
      ::frc971::control_loops::drivetrain::OutputProto* output,
      ::frc971::control_loops::drivetrain::StatusProto* status);

  void Zero(::frc971::control_loops::drivetrain::OutputProto* output);

  ::frc971::control_loops::drivetrain::GoalQueue::QueueReader goal_queue_;
  ::frc971::control_loops::drivetrain::InputQueue::QueueReader input_queue_;
  ::frc971::control_loops::drivetrain::OutputQueue* output_queue_;
  ::frc971::control_loops::drivetrain::StatusQueue* status_queue_;

  ::muan::wpilib::DriverStationQueue::QueueReader driver_station_queue_;
  ::muan::wpilib::gyro::GyroQueue::QueueReader gyro_queue_;

  double last_gyro_heading_ = 0.0;
  double last_gyro_rate_ = 0.0;

  const DrivetrainConfig dt_config_;

  StateFeedbackLoop<7, 2, 3> kf_;
  PolyDrivetrain dt_openloop_;
  DrivetrainMotorsSS dt_closedloop_;

  // Current gears for each drive side.
  Gear left_gear_;
  Gear right_gear_;

  double last_left_voltage_ = 0;
  double last_right_voltage_ = 0;

  double integrated_kf_heading_ = 0;

  bool left_high_requested_;
  bool right_high_requested_;

  bool has_been_enabled_ = false;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_H_
