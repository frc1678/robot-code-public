#ifndef C2019_SUBSYSTEMS_SUPERSTRUCTURE_ELEVATOR_QUEUE_TYPES_H_
#define C2019_SUBSYSTEMS_SUPERSTRUCTURE_ELEVATOR_QUEUE_TYPES_H_

#include "c2019/subsystems/superstructure/elevator/elevator.pb.h"
#include "muan/proto/stack_proto.h"

namespace c2019 {

namespace elevator {

using ElevatorOutputProto =
    muan::proto::StackProto<ElevatorOutput, 256>;
using ElevatorInputProto =
    muan::proto::StackProto<ElevatorInput, 256>;
using ElevatorStatusProto =
    muan::proto::StackProto<ElevatorStatus, 256>;
using ElevatorGoalProto =
    muan::proto::StackProto<ElevatorGoal, 256>;

}  // namespace elevator

}  // namespace c2019

#endif  // C2019_SUBSYSTEMS_SUPERSTRUCTURE_ELEVATOR_QUEUE_TYPES_H_
