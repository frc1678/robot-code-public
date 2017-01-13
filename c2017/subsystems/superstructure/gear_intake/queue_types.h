#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_GEAR_INTAKE_QUEUETYPES_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_GEAR_INTAKE_QUEUETYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "c2017/subsystems/superstructure/gear_intake/intake.pb.h"

namespace c2017 {

namespace gear_intake {

using GearIntakeOutputProto = muan::proto::StackProto<GearIntakeOutput, 256>;
using GearIntakeInputProto = muan::proto::StackProto<GearIntakeInput, 256>;
using GearIntakeStatusProto = muan::proto::StackProto<GearIntakeStatus, 256>;
using GearIntakeGoalProto = muan::proto::StackProto<GearIntakeGoal, 256>;

using GearIntakeOutputQueue = muan::queues::MessageQueue<GearIntakeOutputProto, 200>;
using GearIntakeInputQueue = muan::queues::MessageQueue<GearIntakeInputProto, 200>;
using GearIntakeStatusQueue = muan::queues::MessageQueue<GearIntakeStatusProto, 200>;
using GearIntakeGoalQueue = muan::queues::MessageQueue<GearIntakeGoalProto, 200>;

} //gear_intake

} //c2017
#endif
