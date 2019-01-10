#ifndef C2019_AUTONOMOUS_QUEUE_TYPES_H_
#define C2019_AUTONOMOUS_QUEUE_TYPES_H_

#include "c2019/autonomous/autonomous.pb.h"
#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"

namespace c2019 {
namespace autonomous {

using AutoStatusProto = muan::proto::StackProto<AutonomousStatus, 1024>;
using AutoStatusQueue = muan::queues::MessageQueue<AutoStatusProto>;

}
}

#endif  // C2019_AUTONOMOUS_QUEUE_TYPES_H_
