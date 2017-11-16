#ifndef GENERIC_ROBOT_QUEUE_MANAGER_QUEUE_MANAGER_H_
#define GENERIC_ROBOT_QUEUE_MANAGER_QUEUE_MANAGER_H_

#include "muan/logging/logger.h"
#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "muan/teleop/queue_types.h"
#include "muan/webdash/server.h"
#include "muan/wpilib/gyro/queue_types.h"
#include "muan/wpilib/queue_types.h"

#include "third_party/frc971/control_loops/drivetrain/queue_types.h"

using muan::queues::MessageQueue;

namespace generic_robot {

// A class that contains all of the queues, and allows anyone to get a pointer
// to any queue. This is to avoid having all the queues as global variables
// (because that would be gross). Instead, we can just have an instance of this
// as a global, which is much less sketchy.
class QueueManager {
 public:
  static QueueManager* GetInstance();

  void StartLogging();

  // Note: This needs to be the same as the actual message queue in the
  // PdpWrapper class. If you change that, you will need to change this.
  // It is like this to avoid making QueueManager rely on WPILib.
  MessageQueue<muan::proto::StackProto<PdpStatus, 512>>* pdp_status_queue();
  muan::wpilib::DriverStationQueue* driver_station_queue();

  muan::wpilib::gyro::GyroQueue* gyro_queue();

  muan::teleop::JoystickStatusQueue* manipulator_status_queue();
  muan::teleop::JoystickStatusQueue* wheel_status_queue();
  muan::teleop::JoystickStatusQueue* throttle_status_queue();
  muan::teleop::XBoxRumbleQueue* xbox_rumble_queue();

  frc971::control_loops::drivetrain::GoalQueue* drivetrain_goal_queue();
  frc971::control_loops::drivetrain::InputQueue* drivetrain_input_queue();
  frc971::control_loops::drivetrain::OutputQueue* drivetrain_output_queue();
  frc971::control_loops::drivetrain::StatusQueue* drivetrain_status_queue();

  void Reset();

 private:
  QueueManager() = default;
  ~QueueManager() = default;

  MessageQueue<muan::proto::StackProto<PdpStatus, 512>> pdp_status_queue_;
  muan::wpilib::DriverStationQueue driver_station_queue_;

  muan::wpilib::gyro::GyroQueue gyro_queue_;

  muan::teleop::JoystickStatusQueue manipulator_status_queue_;
  muan::teleop::JoystickStatusQueue wheel_status_queue_;
  muan::teleop::JoystickStatusQueue throttle_status_queue_;
  muan::teleop::XBoxRumbleQueue xbox_rumble_queue_;

  frc971::control_loops::drivetrain::GoalQueue drivetrain_goal_queue_;
  frc971::control_loops::drivetrain::InputQueue drivetrain_input_queue_;
  frc971::control_loops::drivetrain::OutputQueue drivetrain_output_queue_;
  frc971::control_loops::drivetrain::StatusQueue drivetrain_status_queue_;

#ifndef FRC1678_NO_QUEUE_LOGGING
  muan::logging::Logger logger_;
  std::thread logger_thread_{std::ref(logger_)};
#endif  // FRC1678_NO_QUEUE_LOGGING

  muan::webdash::WebDashRunner webdash_;
};

}  // namespace generic_robot

#endif  // GENERIC_ROBOT_QUEUE_MANAGER_QUEUE_MANAGER_H_
