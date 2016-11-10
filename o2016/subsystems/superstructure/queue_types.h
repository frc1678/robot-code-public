#ifndef O2016_SUPERSTRUCTURESTATEMACHINE_QUEUETYPES_H_
#define O2016_SUPERSTRUCTURESTATEMACHINE_QUEUETYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "o2016/subsystems/superstructure/superstructure.pb.h"

namespace o2016 {

using SuperstructureGoalProto = muan::proto::StackProto<::o2016::superstructure::SuperstructureGoal, 100>;

}

#endif  // O2016_SUBSYSTEMS_SUPERSTRUCTURE_TURRET_QUEUETYPES_H_
