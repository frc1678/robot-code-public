#ifndef C2019_SUBSYSTEMS_LIMELIGHT_QUEUE_TYPES_H_
#define C2019_SUBSYSTEMS_LIMELIGHT_QUEUE_TYPES_H_

#include "c2019/subsystems/limelight/limelight.pb.h"
#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"

namespace c2019 {
namespace limelight {

using LimelightStatusProto = muan::proto::StackProto<LimelightStatus, 1024>;
using LimelightStatusQueue = muan::queues::MessageQueue<LimelightStatusProto>;

using LimelightGoalProto = muan::proto::StackProto<LimelightGoal, 1024>;
using LimelightGoalQueue = muan::queues::MessageQueue<LimelightGoalProto>;

}  // namespace limelight
}  // namespace c2019

#endif  // C2019_SUBSYSTEMS_LIMELIGHT_QUEUE_TYPES_H_
