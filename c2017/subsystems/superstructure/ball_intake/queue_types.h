#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_BALLINTAKE_QUEUETYPES_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_BALLINTAKE_QUEUETYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "c2017/subsystems/superstructure/intake/intake.pb.h"

namespace c2017 {

namespace ball_intake {

using BallIntakeOutputProto = muan::proto::StackProto<BallIntakeOutput, 256>;
using BallIntakeIntputProto = muan::proto::StackProto<BallIntakeInput, 256>;
using BallIntakeStatusProto = muan::proto::StackProto<BallIntakeStatus, 256>;
using BallIntakeGoalProto = muan::proto::StackProto<BallIntakeGoal, 256>;
  
using BallIntakeOutputQueue = muan::queues::MessageQueue<BallIntakeOutputProto, 100>;
using BallIntakeInputQueue = muan::proto::StackProto<BallIntakeInputProto, 100>;
using BallIntakeStatusQueue = muan::queues::MessageQueue<BallIntakeStatusProto, 100>;
using BallIntakeGoalQueue = muan::queues::MessageQueue<BallIntakeGoalProto, 100>;
}
} 
#endif//C2017_SUBSYSTEMS_SUPERSTRUCTURE_BALLINTAKE_QUEUETYPES_H_
