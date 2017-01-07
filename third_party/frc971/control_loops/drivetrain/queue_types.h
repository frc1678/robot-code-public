#ifndef THIRD_PARTY_FRC971_CONTROL_LOOPS_DRIVETRAIN_QUEUE_TYPES_H_
#define THIRD_PARTY_FRC971_CONTROL_LOOPS_DRIVETRAIN_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "third_party/frc971/control_loops/drivetrain/drivetrain.pb.h"

namespace frc971 {

namespace control_loops {

namespace drivetrain {

using GoalProto = muan::proto::StackProto<DrivetrainGoal, 1024>;
using InputProto = muan::proto::StackProto<DrivetrainInput, 1024>;
using OutputProto = muan::proto::StackProto<DrivetrainOutput, 1024>;
using StatusProto = muan::proto::StackProto<DrivetrainStatus, 1024>;

using GoalQueue = muan::queues::MessageQueue<GoalProto, 200>;
using InputQueue = muan::queues::MessageQueue<InputProto, 200>;
using OutputQueue = muan::queues::MessageQueue<OutputProto, 200>;
using StatusQueue = muan::queues::MessageQueue<StatusProto, 200>;

}  // namespace frc971

}  // namespace control_loops

}  // namespace drivetrain

#endif  // THIRD_PARTY_FRC971_CONTROL_LOOPS_DRIVETRAIN_QUEUE_TYPES_H_
