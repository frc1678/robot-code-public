#ifndef O2016_QUEUE_MANAGER_QUEUE_MANAGER_H_
#define O2016_QUEUE_MANAGER_QUEUE_MANAGER_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"

#include "muan/wpilib/queue_types.h"

#include "o2016/subsystems/drivetrain/queue_types.h"
#include "o2016/subsystems/superstructure/catapult/queue_types.h"
#include "o2016/subsystems/superstructure/intake/queue_types.h"
#include "o2016/subsystems/superstructure/secondaries/queue_types.h"
#include "o2016/subsystems/superstructure/turret/queue_types.h"

using muan::queues::MessageQueue;

namespace o2016 {

// A class that contains all of the queues, and allows anyone to get a reference
// to any queue. This is to avoid having all the queues as global variables
// (because that would be gross). Instead, we can just have an instance of this
// as a global, which is much less sketchy.
class QueueManager {
 public:
  static QueueManager& GetInstance();

  // Note: This needs to be the same as the actual message queue in the
  // PdpWrapper class. If you change that, you will need to change this.
  // It is like this to avoid making QueueManager rely on WPILib.
  MessageQueue<muan::proto::StackProto<PdpStatus, 512>>& pdp_status_queue();
  muan::wpilib::DriverStationQueue& driver_station_queue();

  o2016::turret::TurretInputQueue& turret_input_queue();
  o2016::turret::TurretGoalQueue& turret_goal_queue();
  o2016::turret::TurretStatusQueue& turret_status_queue();
  o2016::turret::TurretOutputQueue& turret_output_queue();

  o2016::drivetrain::DrivetrainInputQueue& drivetrain_input_queue();
  o2016::drivetrain::DrivetrainGoalQueue& drivetrain_goal_queue();
  o2016::drivetrain::DrivetrainStatusQueue& drivetrain_status_queue();
  o2016::drivetrain::DrivetrainOutputQueue& drivetrain_output_queue();

  o2016::catapult::CatapultInputQueue& catapult_input_queue();
  o2016::catapult::CatapultGoalQueue& catapult_goal_queue();
  o2016::catapult::CatapultStatusQueue& catapult_status_queue();
  o2016::catapult::CatapultOutputQueue& catapult_output_queue();

  o2016::intake::IntakeInputQueue& intake_input_queue();
  o2016::intake::IntakeGoalQueue& intake_goal_queue();
  o2016::intake::IntakeStatusQueue& intake_status_queue();
  o2016::intake::IntakeOutputQueue& intake_output_queue();

  o2016::secondaries::SecondariesOutputQueue& secondaries_output_queue();

 private:
  QueueManager() = default;
  ~QueueManager() = default;

  MessageQueue<muan::proto::StackProto<PdpStatus, 512>> pdp_status_queue_;
  muan::wpilib::DriverStationQueue driver_station_queue_;

  o2016::turret::TurretInputQueue turret_input_queue_;
  o2016::turret::TurretGoalQueue turret_goal_queue_;
  o2016::turret::TurretStatusQueue turret_status_queue_;
  o2016::turret::TurretOutputQueue turret_output_queue_;

  o2016::drivetrain::DrivetrainInputQueue drivetrain_input_queue_;
  o2016::drivetrain::DrivetrainGoalQueue drivetrain_goal_queue_;
  o2016::drivetrain::DrivetrainStatusQueue drivetrain_status_queue_;
  o2016::drivetrain::DrivetrainOutputQueue drivetrain_output_queue_;

  o2016::catapult::CatapultInputQueue catapult_input_queue_;
  o2016::catapult::CatapultGoalQueue catapult_goal_queue_;
  o2016::catapult::CatapultStatusQueue catapult_status_queue_;
  o2016::catapult::CatapultOutputQueue catapult_output_queue_;

  o2016::intake::IntakeInputQueue intake_input_queue_;
  o2016::intake::IntakeGoalQueue intake_goal_queue_;
  o2016::intake::IntakeStatusQueue intake_status_queue_;
  o2016::intake::IntakeOutputQueue intake_output_queue_;

  o2016::secondaries::SecondariesOutputQueue secondaries_output_queue_;
};

}  // namespace o2016

#endif  // O2016_QUEUE_MANAGER_QUEUE_MANAGER_H_
