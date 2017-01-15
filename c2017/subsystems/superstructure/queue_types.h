#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_SHOOTER_GROUP_QUEUE_TYPES_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_SHOOTER_GROUP_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "c2017/subsystems/superstructure/shooter_group/shooter_group.pb.h"

namespace c2017 {

namespace shooter_group {

  using ShooterGroupGoalProto = muan::proto::StackProto<::c2017::shooter_group::ShooterGroupGoal, 256>;

  using ShooterGroupGoalQueue = muan::queues::MessageQueue<ShooterGroupGoalProto, 200>;

} // namespace shooter_group

} // namespace c2017

#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_SHOOTER_GROUP_QUEUE_TYPES_H_
