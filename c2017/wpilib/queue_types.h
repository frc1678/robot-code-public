#ifndef C2017_WPILIB_QUEUE_TYPES_H_
#define C2017_WPILIB_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "c2017/wpilib/wpilib_superstructure.pb.h"

namespace c2017 {

namespace wpilib {

  using WpilibOutputProto = muan::proto::StackProto<WpilibOutput, 1024>;

  using WpilibOutputQueue = muan::queues::MessageQueue<WpilibOutputProto, 1024>;

} // wpilib

} // c2017

#endif
