#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_QUEUETYPES_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_QUEUETYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "c2017/subsystems/superstructure/intake_group/intake_group.pb.h"

namespace c2017 {
  
  namespace intake_group {

  using IntakeGroupGoalProto = muan::proto::StackProto<IntakeGoal, 256>;

  using IntakeGroupGoalQueue = muan::queues::MessageQueue<IntakeGoalProto, 200>;
  }
}
#endif//C2017_SUBSYSTEMS_SUPERSTRUCTURE_QUEUETYPES_H_
