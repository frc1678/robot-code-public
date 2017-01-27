#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_CLIMBER_QUEUE_TYPES_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_CLIMBER_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "c2017/subsystems/superstructure/climber/climber.pb.h"

namespace c2017 {
namespace climber {

using ClimberOutputProto = muan::proto::StackProto<ClimberOutput, 1024>;
using ClimberInputProto = muan::proto::StackProto<ClimberInput, 1024>;
using ClimberStatusProto = muan::proto::StackProto<ClimberStatus, 1024>;
using ClimberGoalProto = muan::proto::StackProto<ClimberGoal, 1024>;

using ClimberGoalQueue = muan::queues::MessageQueue<ClimberGoalProto, 100>;
using ClimberInputQueue = muan::queues::MessageQueue<ClimberInputProto, 100>;
using ClimberStatusQueue = muan::queues::MessageQueue<ClimberStatusProto, 100>;

}  // namespace climber
}  // namespace c2017

#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_CLIMBER_QUEUE_TYPES_H_
