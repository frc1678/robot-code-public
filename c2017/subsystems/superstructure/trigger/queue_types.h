#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_TRIGGER_QUEUETYPES_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_TRIGGER_QUEUETYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "c2017/subsystems/superstructure/trigger/trigger.pb.h"

namespace c2017 {

namespace trigger {

using TriggerGoalProto = muan::proto::StackProto<::c2017::trigger::TriggerGoal, 256>;
using TriggerInputProto = muan::proto::StackProto<::c2017::trigger::TriggerInput, 256>;
using TriggerStatusProto = muan::proto::StackProto<::c2017::trigger::TriggerStatus, 256>;
using TriggerOutputProto = muan::proto::StackProto<::c2017::trigger::TriggerOutput, 256>;

using TriggerInputQueue = muan::queues::MessageQueue<TriggerInputProto, 200>;
using TriggerGoalQueue = muan::queues::MessageQueue<TriggerGoalProto, 200>;
using TriggerOutputQueue = muan::queues::MessageQueue<TriggerOutputProto, 200>;
using TriggerStatusQueue = muan::queues::MessageQueue<TriggerStatusProto, 200>;

}  // namespace trigger

}  // namespace c2017

#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_TRIGGER_QUEUETYPES_H_
