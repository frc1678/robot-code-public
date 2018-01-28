#ifndef O2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_GEAR_INTAKE_QUEUE_TYPES_H_
#define O2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_GEAR_INTAKE_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "o2017/subsystems/superstructure/ground_gear_intake/ground_gear_intake.pb.h"

namespace o2017 {

namespace ground_gear_intake {

using GroundGearIntakeOutputProto =
    muan::proto::StackProto<GroundGearIntakeOutput, 256>;
using GroundGearIntakeInputProto =
    muan::proto::StackProto<GroundGearIntakeInput, 256>;
using GroundGearIntakeStatusProto =
    muan::proto::StackProto<GroundGearIntakeStatus, 256>;
using GroundGearIntakeGoalProto =
    muan::proto::StackProto<GroundGearIntakeGoal, 256>;

using GroundGearIntakeInputQueue =
    muan::queues::MessageQueue<GroundGearIntakeInputProto>;
using GroundGearIntakeStatusQueue =
    muan::queues::MessageQueue<GroundGearIntakeStatusProto>;
using GroundGearIntakeOutputQueue =
    muan::queues::MessageQueue<GroundGearIntakeOutputProto>;

}  // namespace ground_gear_intake
}  // namespace o2017

#endif  // O2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_GEAR_INTAKE_QUEUE_TYPES_H_
