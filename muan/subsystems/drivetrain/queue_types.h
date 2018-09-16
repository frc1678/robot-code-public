#ifndef MUAN_SUBSYSTEMS_DRIVETRAIN_QUEUE_TYPES_H_
#define MUAN_SUBSYSTEMS_DRIVETRAIN_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "muan/subsystems/drivetrain/drivetrain.pb.h"

namespace muan {
namespace subsystems {
namespace drivetrain {

using GoalProto = muan::proto::StackProto<DrivetrainGoal, 1024>;
using InputProto = muan::proto::StackProto<DrivetrainInput, 1024>;
using OutputProto = muan::proto::StackProto<DrivetrainOutput, 1024>;
using StatusProto = muan::proto::StackProto<DrivetrainStatus, 1024>;

using GoalQueue = muan::queues::MessageQueue<GoalProto>;
using InputQueue = muan::queues::MessageQueue<InputProto>;
using OutputQueue = muan::queues::MessageQueue<OutputProto>;
using StatusQueue = muan::queues::MessageQueue<StatusProto>;

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace muan

#endif  //  MUAN_SUBSYSTEMS_DRIVETRAIN_QUEUE_TYPES_H_
