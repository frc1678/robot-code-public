#ifndef C2019_SUBSYSTEMS_SUPERSTRUCTURE_HATCH_INTAKE_QUEUE_TYPES_H_
#define C2019_SUBSYSTEMS_SUPERSTRUCTURE_HATCH_INTAKE_QUEUE_TYPES_H_

#include "c2019/subsystems/superstructure/hatch_intake/hatch_intake.pb.h"
#include "muan/proto/stack_proto.h"

namespace c2019 {

namespace hatch_intake {

using HatchIntakeOutputProto =
    muan::proto::StackProto<HatchIntakeOutput, 256>;
using HatchIntakeInputProto =
    muan::proto::StackProto<HatchIntakeInput, 256>;
using HatchIntakeStatusProto =
    muan::proto::StackProto<HatchIntakeStatus, 256>;
using HatchIntakeGoalProto =
    muan::proto::StackProto<HatchIntakeGoal, 256>;

}  // namespace hatch_intake

}  // namespace c2019

#endif  // C2019_SUBSYSTEMS_SUPERSTRUCTURE_HATCH_INTAKE_QUEUE_TYPES_H_
