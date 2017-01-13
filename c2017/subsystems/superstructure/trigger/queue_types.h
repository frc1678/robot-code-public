#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_TRIGGER_QUEUETYPES_H_
#define C2016_SUBSYSTEMS_SUPERSTRUCTURE_TRIGGER_QUEUETYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "o2017/subsystems/superstructure/trigger/trigger.pb.h"

namespace o2017 {

namespace trigger {

using TriggerGoalProto = muan::proto::StackProto<::o2017::trigger::TriggerGoal, 256>;
using TriggerInputProto = muan::proto::StackProto<::o2017::trigger::TriggerInput, 256>;
using TriggerStatusProto = muan::proto::StackProto<::o2017::trigger::TriggerStatus, 256>;
using TriggerOutputProto = muan::proto::StackProto<::o2017::trigger::TriggerOutput, 256>;

using TriggerInputQueue = muan::queues::MessageQueue<TriggerInputProto, 200>;
using TriggerGoalQueue = muan::queues::MessageQueue<TriggerGoalProto, 200>;
using TriggerOutputQueue = muan::queues::MessageQueue<TriggerOutputProto, 200>;
using TriggerStatusQueue = muan::queues::MessageQueue<TriggerStatusProto, 200>;
}
}

#endif  // C2016_SUBSYSTEMS_SUPERSTRUCTURE_TRIGGER_QUEUETYPES_H_
