#ifndef C2017_WPILIB_QUEUE_TYPES_H_
#define C2017_WPILIB_QUEUE_TYPES_H_

#include "c2017/wpilib/wpilib_superstructure.pb.h"
#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"

namespace c2017 {

namespace wpilib {

using WpilibOutputProto = muan::proto::StackProto<WpilibOutput, 1024>;

using WpilibOutputQueue = muan::queues::MessageQueue<WpilibOutputProto>;

}  // namespace wpilib

}  // namespace c2017

#endif  // C2017_WPILIB_QUEUE_TYPES_H_
