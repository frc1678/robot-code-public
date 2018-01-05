#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_BALL_INTAKE_QUEUE_TYPES_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_BALL_INTAKE_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "c2017/subsystems/superstructure/ground_ball_intake/ground_ball_intake.pb.h"

namespace c2017 {

namespace ground_ball_intake {

using GroundBallIntakeOutputProto = muan::proto::StackProto<GroundBallIntakeOutput, 256>;
using GroundBallIntakeStatusProto = muan::proto::StackProto<GroundBallIntakeStatus, 256>;
using GroundBallIntakeGoalProto = muan::proto::StackProto<GroundBallIntakeGoal, 256>;

using GroundBallIntakeStatusQueue = muan::queues::MessageQueue<GroundBallIntakeStatusProto>;

}  // namespace ground_ball_intake

}  // namespace c2017

#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_BALL_INTAKE_QUEUE_TYPES_H_
