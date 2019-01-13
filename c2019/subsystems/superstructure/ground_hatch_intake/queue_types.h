#ifndef C2019_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_HATCH_INTAKE_QUEUE_TYPES_H_
#define C2019_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_HATCH_INTAKE_QUEUE_TYPES_H_

#include "c2019/subsystems/superstructure/ground_hatch_intake/ground_hatch_intake.pb.h"
#include "muan/proto/stack_proto.h"

namespace c2019 {

namespace ground_hatch_intake {

using GroundHatchIntakeOutputProto =
    muan::proto::StackProto<GroundHatchIntakeOutput, 256>;
using GroundHatchIntakeInputProto =
    muan::proto::StackProto<GroundHatchIntakeInput, 256>;
using GroundHatchIntakeStatusProto =
    muan::proto::StackProto<GroundHatchIntakeStatus, 256>;
using GroundHatchIntakeGoalProto =
    muan::proto::StackProto<GroundHatchIntakeGoal, 256>;

}  // namespace ground_hatch_intake

}  // namespace c2019

#endif  // C2019_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_HATCH_INTAKE_QUEUE_TYPES_H_
