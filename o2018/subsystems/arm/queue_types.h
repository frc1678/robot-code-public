#ifndef O2018_SUBSYSTEMS_ARM_QUEUE_TYPES_H_
#define O2018_SUBSYSTEMS_ARM_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "o2018/subsystems/arm/arm.pb.h"

namespace o2018 {
namespace subsystems {
namespace arm {

using ArmInputProto = muan::proto::StackProto<ArmInput, 256>;
using ArmOutputProto = muan::proto::StackProto<ArmOutput, 256>;
using ArmGoalProto = muan::proto::StackProto<ArmGoal, 256>;
using ArmStatusProto = muan::proto::StackProto<ArmStatus, 256>;

using ArmInputQueue = muan::queues::MessageQueue<ArmInputProto>;
using ArmOutputQueue = muan::queues::MessageQueue<ArmOutputProto>;
using ArmGoalQueue = muan::queues::MessageQueue<ArmGoalProto>;
using ArmStatusQueue = muan::queues::MessageQueue<ArmStatusProto>;

}  // namespace arm
}  // namespace subsystems
}  // namespace o2018

#endif  // O2018_SUBSYSTEMS_ARM_QUEUE_TYPES_H_
