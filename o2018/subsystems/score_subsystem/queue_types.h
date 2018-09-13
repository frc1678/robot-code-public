#ifndef O2018_SUBSYSTEMS_SCORE_SUBSYSTEM_QUEUE_TYPES_H_
#define O2018_SUBSYSTEMS_SCORE_SUBSYSTEM_QUEUE_TYPES_H_

#include "o2018/subsystems/score_subsystem/score_subsystem.pb.h"
#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"

namespace o2018 {
namespace score_subsystem {

using ScoreSubsystemInputProto =
    muan::proto::StackProto<ScoreSubsystemInput, 256>;
using ScoreSubsystemOutputProto =
    muan::proto::StackProto<ScoreSubsystemOutput, 256>;
using ScoreSubsystemGoalProto =
    muan::proto::StackProto<ScoreSubsystemGoal, 256>;
using ScoreSubsystemStatusProto =
    muan::proto::StackProto<ScoreSubsystemStatus, 256>;

using ScoreSubsystemInputQueue =
    muan::queues::MessageQueue<ScoreSubsystemInputProto>;
using ScoreSubsystemStatusQueue =
    muan::queues::MessageQueue<ScoreSubsystemStatusProto>;
using ScoreSubsystemGoalQueue =
    muan::queues::MessageQueue<ScoreSubsystemGoalProto>;
using ScoreSubsystemOutputQueue =
    muan::queues::MessageQueue<ScoreSubsystemOutputProto>;
using ScoreSubsystemGoalQueue =
    muan::queues::MessageQueue<ScoreSubsystemGoalProto>;

}  // namespace score_subsystem
}  // namespace o2018

#endif  // O2018_SUBSYSTEMS_SCORE_SUBSYSTEM_QUEUE_TYPES_H_
