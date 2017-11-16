#ifndef C2017_QUEUE_MANAGER_QUEUE_MANAGER_H_
#define C2017_QUEUE_MANAGER_QUEUE_MANAGER_H_

#include <thread>
#include <functional>
#include <vector>
#include <string>
#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "muan/logging/logger.h"

#include "muan/wpilib/gyro/queue_types.h"
#include "muan/wpilib/queue_types.h"
#include "muan/teleop/queue_types.h"

#include "muan/webdash/queue_types.h"
#include "muan/webdash/server.h"

#include "third_party/frc971/control_loops/drivetrain/queue_types.h"
#include "c2017/subsystems/superstructure/ground_ball_intake/queue_types.h"
#include "c2017/subsystems/superstructure/shooter/queue_types.h"
#include "c2017/subsystems/superstructure/magazine/queue_types.h"
#include "c2017/subsystems/superstructure/ground_gear_intake/queue_types.h"
#include "c2017/subsystems/superstructure/climber/queue_types.h"
#include "c2017/subsystems/superstructure/queue_types.h"
#include "c2017/vision/queue_types.h"
#include "c2017/wpilib/queue_types.h"
#include "c2017/subsystems/lights/queue_types.h"

using muan::queues::MessageQueue;

namespace c2017 {

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
  MessageQueue<muan::proto::StackProto<PdpStatus, 512>>* pdp_status_queue();
  muan::wpilib::DriverStationQueue* driver_station_queue();

  muan::wpilib::gyro::GyroQueue* gyro_queue();

  frc971::control_loops::drivetrain::GoalQueue* drivetrain_goal_queue();
  frc971::control_loops::drivetrain::InputQueue* drivetrain_input_queue();
  frc971::control_loops::drivetrain::OutputQueue* drivetrain_output_queue();
  frc971::control_loops::drivetrain::StatusQueue* drivetrain_status_queue();

  c2017::wpilib::WpilibOutputQueue* superstructure_output_queue();
  c2017::superstructure::SuperstructureStatusQueue* superstructure_status_queue();

  c2017::ground_ball_intake::GroundBallIntakeStatusQueue* ground_ball_intake_status_queue();

  c2017::shooter::ShooterInputQueue* shooter_input_queue();
  c2017::shooter::ShooterStatusQueue* shooter_status_queue();

  c2017::magazine::MagazineStatusQueue* magazine_status_queue();

  c2017::ground_gear_intake::GroundGearIntakeInputQueue* ground_gear_input_queue();
  c2017::ground_gear_intake::GroundGearIntakeStatusQueue* ground_gear_status_queue();
  c2017::ground_gear_intake::GroundGearIntakeOutputQueue* ground_gear_output_queue();

  c2017::ground_ball_intake::GroundBallIntakeStatusQueue* ball_intake_status_queue();

  c2017::climber::ClimberInputQueue* climber_input_queue();
  c2017::climber::ClimberStatusQueue* climber_status_queue();

  c2017::vision::VisionInputQueue* vision_input_queue();
  c2017::vision::VisionStatusQueue* vision_status_queue();
  c2017::vision::VisionGoalQueue* vision_goal_queue();

  c2017::intake_group::IntakeGroupGoalQueue* intake_group_goal_queue();
  c2017::shooter_group::ShooterGroupGoalQueue* shooter_group_goal_queue();

  c2017::lights::LightsOutputQueue* lights_output_queue();

  muan::teleop::JoystickStatusQueue* manipulator_status_queue();
  muan::teleop::JoystickStatusQueue* wheel_status_queue();
  muan::teleop::JoystickStatusQueue* throttle_status_queue();
  muan::teleop::XBoxRumbleQueue* xbox_rumble_queue();
  
  const std::vector<std::string> auto_list_;
  
  void Reset();

 private:
  QueueManager() = default;
  ~QueueManager() = default;

  MessageQueue<muan::proto::StackProto<PdpStatus, 512>> pdp_status_queue_;
  muan::wpilib::DriverStationQueue driver_station_queue_;

  muan::wpilib::gyro::GyroQueue gyro_queue_;

  frc971::control_loops::drivetrain::GoalQueue drivetrain_goal_queue_;
  frc971::control_loops::drivetrain::InputQueue drivetrain_input_queue_;
  frc971::control_loops::drivetrain::OutputQueue drivetrain_output_queue_;
  frc971::control_loops::drivetrain::StatusQueue drivetrain_status_queue_;

  c2017::wpilib::WpilibOutputQueue superstructure_output_queue_;
  c2017::superstructure::SuperstructureStatusQueue superstructure_status_queue_;

  c2017::shooter::ShooterInputQueue shooter_input_queue_;
  c2017::shooter::ShooterStatusQueue shooter_status_queue_;

  c2017::magazine::MagazineStatusQueue magazine_status_queue_;

  c2017::ground_gear_intake::GroundGearIntakeInputQueue ground_gear_input_queue_;
  c2017::ground_gear_intake::GroundGearIntakeStatusQueue ground_gear_status_queue_;
  c2017::ground_gear_intake::GroundGearIntakeOutputQueue ground_gear_output_queue_;

  c2017::ground_ball_intake::GroundBallIntakeStatusQueue ground_ball_intake_status_queue_;

  c2017::climber::ClimberGoalQueue climber_goal_queue_;
  c2017::climber::ClimberInputQueue climber_input_queue_;
  c2017::climber::ClimberStatusQueue climber_status_queue_;

  c2017::vision::VisionInputQueue vision_input_queue_;
  c2017::vision::VisionStatusQueue vision_status_queue_;
  c2017::vision::VisionGoalQueue vision_goal_queue_;

  c2017::intake_group::IntakeGroupGoalQueue intake_group_goal_queue_;
  c2017::shooter_group::ShooterGroupGoalQueue shooter_group_goal_queue_;

  c2017::lights::LightsOutputQueue lights_output_queue_;

  muan::teleop::JoystickStatusQueue manipulator_status_queue_;
  muan::teleop::JoystickStatusQueue wheel_status_queue_;
  muan::teleop::JoystickStatusQueue throttle_status_queue_;
  muan::teleop::XBoxRumbleQueue xbox_rumble_queue_;

#ifndef FRC1678_NO_QUEUE_LOGGING
  muan::logging::Logger logger_;
  std::thread logger_thread_{std::ref(logger_)};
#endif  // FRC1678_NO_QUEUE_LOGGING

  muan::webdash::WebDashRunner webdash_;
};

}  // namespace c2017

#endif  // C2017_QUEUE_MANAGER_QUEUE_MANAGER_H_
