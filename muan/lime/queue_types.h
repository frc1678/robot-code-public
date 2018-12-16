#ifndef MUAN_LIME_QUEUE_TYPES_H_
#define MUAN_LIME_QUEUE_TYPES_H_

#include "muan/lime/lime.pb.h"
#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"

namespace muan {
namespace lime {

using LimeStatusProto = muan::proto::StackProto<LimeStatus, 1024>;
using LimeStatusQueue = muan::queues::MessageQueue<LimeStatusProto>;

using LimeGoalProto = muan::proto::StackProto<LimeGoal, 1024>;
using LimeGoalQueue = muan::queues::MessageQueue<LimeGoalProto>;

}  // namespace lime
}  // namespace muan

#endif  // MUAN_LIME_QUEUE_TYPES_H_
