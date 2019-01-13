#ifndef C2019_SUBSYSTEMS_SUPERSTRUCTURE_QUEUE_TYPES_H_
#define C2019_SUBSYSTEMS_SUPERSTRUCTURE_QUEUE_TYPES_H_

#include "c2019/subsystems/superstructure/superstructure.pb.h"
#include "muan/proto/stack_proto.h"

namespace c2019 {

namespace superstructure {

using SuperstructureOutputProto =
    muan::proto::StackProto<SuperstructureOutput, 256>;
using SuperstructureInputProto =
    muan::proto::StackProto<SuperstructureInput, 256>;
using SuperstructureStatusProto =
    muan::proto::StackProto<SuperstructureStatus, 256>;
using SuperstructureGoalProto =
    muan::proto::StackProto<SuperstructureGoal, 256>;

using SuperstructureInputQueue =
    muan::queues::MessageQueue<SuperstructureInputProto>;
using SuperstructureStatusQueue =
    muan::queues::MessageQueue<SuperstructureStatusProto>;
using SuperstructureGoalQueue =
    muan::queues::MessageQueue<SuperstructureGoalProto>;
using SuperstructureOutputQueue =
    muan::queues::MessageQueue<SuperstructureOutputProto>;

}  // namespace superstructure

}  // namespace c2019

#endif  // C2019_SUBSYSTEMS_SUPERSTRUCTURE_QUEUE_TYPES_H_
