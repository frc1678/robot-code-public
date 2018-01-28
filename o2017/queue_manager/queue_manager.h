#ifndef O2017_QUEUE_MANAGER_QUEUE_MANAGER_H_
#define O2017_QUEUE_MANAGER_QUEUE_MANAGER_H_

#include "muan/logging/logger.h"
#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "muan/teleop/queue_types.h"
#include "muan/webdash/server.h"
#include "muan/wpilib/gyro/queue_types.h"
#include "muan/wpilib/queue_types.h"

#include "o2017/subsystems/superstructure/queue_types.h"
#include "third_party/frc971/control_loops/drivetrain/queue_types.h"

using muan::queues::MessageQueue;

namespace o2017 {

// A class that contains all of the queues, and allows anyone to get a reference
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
  MessageQueue<muan::proto::StackProto<PdpStatus, 512>>& pdp_status_queue();
  muan::wpilib::DriverStationQueue& driver_station_queue();

  muan::wpilib::gyro::GyroQueue* gyro_queue();

  frc971::control_loops::drivetrain::GoalQueue* drivetrain_goal_queue();
  frc971::control_loops::drivetrain::InputQueue* drivetrain_input_queue();
  frc971::control_loops::drivetrain::OutputQueue* drivetrain_output_queue();
  frc971::control_loops::drivetrain::StatusQueue* drivetrain_status_queue();

  o2017::superstructure::InputQueue* superstructure_input_queue();
  o2017::superstructure::OutputQueue* superstructure_output_queue();
  o2017::superstructure::StatusQueue* superstructure_status_queue();
  o2017::superstructure::GoalQueue* superstructure_goal_queue();

  void Reset();

 private:
  QueueManager() = default;
  ~QueueManager() = default;

  frc971::control_loops::drivetrain::GoalQueue drivetrain_goal_queue_;
  frc971::control_loops::drivetrain::InputQueue drivetrain_input_queue_;
  frc971::control_loops::drivetrain::OutputQueue drivetrain_output_queue_;
  frc971::control_loops::drivetrain::StatusQueue drivetrain_status_queue_;

  o2017::superstructure::InputQueue superstructure_input_queue_;
  o2017::superstructure::OutputQueue superstructure_output_queue_;
  o2017::superstructure::StatusQueue superstructure_status_queue_;
  o2017::superstructure::GoalQueue superstructure_goal_queue_;

  muan::teleop::JoystickStatusQueue manipulator_status_queue_;
  muan::teleop::JoystickStatusQueue wheel_status_queue_;
  muan::teleop::JoystickStatusQueue throttle_status_queue_;

  MessageQueue<muan::proto::StackProto<PdpStatus, 512>> pdp_status_queue_;
  muan::wpilib::DriverStationQueue driver_station_queue_;

  muan::wpilib::gyro::GyroQueue gyro_queue_;
  muan::logging::Logger logger_;

  muan::webdash::WebDashRunner webdash_;
};

}  // namespace o2017

#endif  // O2017_QUEUE_MANAGER_QUEUE_MANAGER_H_
