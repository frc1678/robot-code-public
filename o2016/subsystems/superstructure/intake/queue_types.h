#ifndef O2016_SUBSYSTEMS_SUPERSTRUCTURE_INTAKE_QUEUETYPES_H_
#define O2016_SUBSYSTEMS_SUPERSTRUCTURE_INTAKE_QUEUETYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "o2016/subsystems/superstructure/intake/intake.pb.h"

namespace o2016 {

namespace intake {

using IntakeOutputProto = muan::proto::StackProto<IntakeOutput, 1024>;
using IntakeInputProto = muan::proto::StackProto<IntakeInput, 1024>;
using IntakeStatusProto = muan::proto::StackProto<IntakeStatus, 1024>;
using IntakeGoalProto = muan::proto::StackProto<IntakeGoal, 1024>;

using IntakeOutputQueue = muan::queues::MessageQueue<IntakeOutputProto, 1024>;
using IntakeInputQueue = muan::queues::MessageQueue<IntakeInputProto, 1024>;
using IntakeStatusQueue = muan::queues::MessageQueue<IntakeStatusProto, 1024>;
using IntakeGoalQueue = muan::queues::MessageQueue<IntakeGoalProto, 1024>;

}

}
#endif
