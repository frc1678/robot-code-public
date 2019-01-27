#ifndef C2019_COMMANDS_QUEUE_TYPES_H_
#define C2019_COMMANDS_QUEUE_TYPES_H_

#include "c2019/commands/commands.pb.h"
#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"

namespace c2019 {
namespace commands {

using AutoGoalProto = muan::proto::StackProto<AutonomousGoal, 1024>;
using AutoGoalQueue = muan::queues::MessageQueue<AutoGoalProto>;

using AutoStatusProto = muan::proto::StackProto<AutonomousStatus, 1024>;
using AutoStatusQueue = muan::queues::MessageQueue<AutoStatusProto>;

}  // namespace commands
}  // namespace c2019

#endif  // C2019_COMMANDS_QUEUE_TYPES_H_
