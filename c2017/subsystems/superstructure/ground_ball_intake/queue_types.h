#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_BALL_INTAKE_QUEUE_TYPES_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_BALL_INTAKE_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "c2017/subsystems/superstructure/ground_ball_intake/ground_ball_intake.pb.h"

namespace c2017 {

namespace ball_intake {

using GroundBallIntakeOutputProto = muan::proto::StackProto<GroundBallIntakeOutput, 256>;
using GroundBallIntakeStatusProto = muan::proto::StackProto<GroundBallIntakeStatus, 256>;
using GroundBallIntakeGoalProto = muan::proto::StackProto<GroundBallIntakeGoal, 256>;

using GroundBallIntakeOutputQueue = muan::queues::MessageQueue<GroundBallIntakeOutputProto, 100>;
using GroundBallIntakeStatusQueue = muan::queues::MessageQueue<GroundBallIntakeStatusProto, 100>;
using GroundBallIntakeGoalQueue = muan::queues::MessageQueue<GroundBallIntakeGoalProto, 100>;

}

}

#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_BALLINTAKE_QUEUETYPES_H_
