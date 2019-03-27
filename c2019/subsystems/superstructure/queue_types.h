#ifndef C2019_SUBSYSTEMS_SUPERSTRUCTURE_QUEUE_TYPES_H_
#define C2019_SUBSYSTEMS_SUPERSTRUCTURE_QUEUE_TYPES_H_

#include "c2019/subsystems/superstructure/superstructure.pb.h"
#include "muan/proto/stack_proto.h"
#include "muan/queues/queue_manager.h"

namespace c2019 {

namespace superstructure {

using SuperstructureOutputProto =
    muan::proto::StackProto<SuperstructureOutput, 1024>;
using SuperstructureInputProto =
    muan::proto::StackProto<SuperstructureInput, 1024>;
using SuperstructureStatusProto =
    muan::proto::StackProto<SuperstructureStatus, 1024>;
using SuperstructureGoalProto =
    muan::proto::StackProto<SuperstructureGoal, 1024>;

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
