#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_SHOOTER_QUEUE_TYPES_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_SHOOTER_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "c2017/subsystems/superstructure/shooter/shooter.pb.h"

namespace c2017 {

namespace shooter {

using ShooterGoalProto = muan::proto::StackProto<::c2017::shooter::ShooterGoal, 256>;
using ShooterInputProto = muan::proto::StackProto<::c2017::shooter::ShooterInput, 256>;
using ShooterStatusProto = muan::proto::StackProto<::c2017::shooter::ShooterStatus, 256>;
using ShooterOutputProto = muan::proto::StackProto<::c2017::shooter::ShooterOutput, 256>;

using ShooterInputQueue = muan::queues::MessageQueue<ShooterInputProto, 200>;
using ShooterStatusQueue = muan::queues::MessageQueue<ShooterStatusProto, 200>;

}  // namespace shooter

}  // namespace c2017

#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_SHOOTER_QUEUE_TYPES_H_
