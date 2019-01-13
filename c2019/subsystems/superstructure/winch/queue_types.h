#ifndef C2019_SUBSYSTEMS_SUPERSTRUCTURE_WINCH_QUEUE_TYPES_H_
#define C2019_SUBSYSTEMS_SUPERSTRUCTURE_WINCH_QUEUE_TYPES_H_

#include "c2019/subsystems/superstructure/winch/winch.pb.h"
#include "muan/proto/stack_proto.h"

namespace c2019 {

namespace winch {

using WinchOutputProto =
    muan::proto::StackProto<WinchOutput, 256>;
using WinchInputProto =
    muan::proto::StackProto<WinchInput, 256>;
using WinchStatusProto =
    muan::proto::StackProto<WinchStatus, 256>;
using WinchGoalProto =
    muan::proto::StackProto<WinchGoal, 256>;

}  // namespace winch

}  // namespace c2019

#endif  // C2019_SUBSYSTEMS_SUPERSTRUCTURE_WINCH_QUEUE_TYPES_H_
