#ifndef C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_QUEUE_TYPES_H_
#define C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "c2018/subsystems/score_subsystem/score_subsystem.pb.h"

namespace c2018 {

namespace score_subsystem {

using ScoreSubsystemInputProto = muan::proto::StackProto<ScoreSubsystemInput, 256>;
using ScoreSubsystemOutputProto = muan::proto::StackProto<ScoreSubsystemOutput, 256>;
using ScoreSubsystemGoalProto = muan::proto::StackProto<ScoreSubsystemGoal, 256>;
using ScoreSubsystemGoalGodModeProto = muan::proto::StackProto<ScoreSubsystemGoalGodMode, 256>;
using ScoreSubsystemStatusProto = muan::proto::StackProto<ScoreSubsystemStatus, 256>;

using ScoreSubsystemInputQueue = muan::queues::MessageQueues<ScoreSubsystemInputProto>;
using ScoreSubsystemStatusQueue = muan::queues::MessageQueues<ScoreSubsystemStatusProto>;
using ScoreSubsystemOutputQueue = muan::queues::MessageQueues<ScoreSubsystemOutputProto>;

}  // namespace score_subsystem
}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_QUEUE_TYPES_H_
