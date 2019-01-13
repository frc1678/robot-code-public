#ifndef C2019_SUBSYSTEMS_SUPERSTRUCTURE_CARGO_INTAKE_QUEUE_TYPES_H_
#define C2019_SUBSYSTEMS_SUPERSTRUCTURE_CARGO_INTAKE_QUEUE_TYPES_H_

#include "c2019/subsystems/superstructure/cargo_intake/cargo_intake.pb.h"
#include "muan/proto/stack_proto.h"

namespace c2019 {

namespace cargo_intake {

using CargoIntakeOutputProto =
    muan::proto::StackProto<CargoIntakeOutput, 256>;
using CargoIntakeInputProto =
    muan::proto::StackProto<CargoIntakeInput, 256>;
using CargoIntakeStatusProto =
    muan::proto::StackProto<CargoIntakeStatus, 256>;
using CargoIntakeGoalProto =
    muan::proto::StackProto<CargoIntakeGoal, 256>;

}  // namespace cargo_intake

}  // namespace c2019

#endif  // C2019_SUBSYSTEMS_SUPERSTRUCTURE_CARGO_INTAKE_QUEUE_TYPES_H_
