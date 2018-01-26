#ifndef O2016_SUBSYSTEMS_SUPERSTRUCTURE_TURRET_QUEUE_TYPES_H_
#define O2016_SUBSYSTEMS_SUPERSTRUCTURE_TURRET_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "o2016/subsystems/superstructure/turret/turret.pb.h"

namespace o2016 {
namespace turret {

using TurretGoalProto =
    muan::proto::StackProto<::o2016::turret::TurretGoal, 100>;
using TurretInputProto =
    muan::proto::StackProto<::o2016::turret::TurretInput, 100>;
using TurretStatusProto =
    muan::proto::StackProto<::o2016::turret::TurretStatus, 100>;
using TurretOutputProto =
    muan::proto::StackProto<::o2016::turret::TurretOutput, 100>;

using TurretInputQueue = muan::queues::MessageQueue<TurretInputProto>;
using TurretGoalQueue = muan::queues::MessageQueue<TurretGoalProto>;
using TurretOutputQueue = muan::queues::MessageQueue<TurretOutputProto>;
using TurretStatusQueue = muan::queues::MessageQueue<TurretStatusProto>;

}  // namespace turret
}  // namespace o2016

#endif  // O2016_SUBSYSTEMS_SUPERSTRUCTURE_TURRET_QUEUE_TYPES_H_
