#ifndef O2017_SUBSYSTEMS_SUPERSTRUCTURE_QUEUE_TYPES_H_
#define O2017_SUBSYSTEMS_SUPERSTRUCTURE_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "o2017/subsystems/superstructure/superstructure.pb.h"

namespace o2017 {

namespace superstructure {

using SuperstructureStatusProto =
    muan::proto::StackProto<SuperstructureStatus, 256>;
using SuperstructureInputProto =
    muan::proto::StackProto<SuperstructureInput, 256>;
using SuperstructureGoalProto =
    muan::proto::StackProto<SuperstructureGoal, 256>;
using SuperstructureOutputProto =
    muan::proto::StackProto<SuperstructureOutput, 256>;

using StatusQueue = muan::queues::MessageQueue<SuperstructureStatusProto>;
using InputQueue = muan::queues::MessageQueue<SuperstructureInputProto>;
using GoalQueue = muan::queues::MessageQueue<SuperstructureGoalProto>;
using OutputQueue = muan::queues::MessageQueue<SuperstructureOutputProto>;

}  // namespace superstructure

}  // namespace o2017

#endif  // O2017_SUBSYSTEMS_SUPERSTRUCTURE_QUEUE_TYPES_H_
