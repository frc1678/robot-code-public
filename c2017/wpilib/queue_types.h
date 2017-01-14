#ifndef C2017_WPILIB_QUEUE_TYPES_H
#define C2017_WPILIB_QUEUE_TYPES_H

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "c2017/wpilibwpilib.pb.h"

namespace c2017 {

namespace wpilib {
  
  using WpilibOutputProto = muan::proto::StackProto<WpilibOutput, 1024>;

  using WpilibOutputQueue = muan::queues::MessageQueue<WpilibOutputProto, 200>; 

} // wpilib

} // c2017

#endif 
