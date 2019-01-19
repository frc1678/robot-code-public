#ifndef C2019_SUBSYSTEMS_SUPERSTRUCTURE_WRIST_QUEUE_TYPES_H_
#define C2019_SUBSYSTEMS_SUPERSTRUCTURE_WRIST_QUEUE_TYPES_H_

#include "c2019/subsystems/superstructure/wrist/wrist.pb.h"
#include "muan/proto/stack_proto.h"

namespace c2019 {
namespace wrist {

using WristOutputProto =
    muan::proto::StackProto<WristOutput, 256>;
using WristInputProto =
    muan::proto::StackProto<WristInput, 256>;
using WristStatusProto =
    muan::proto::StackProto<WristStatus, 256>;
using WristGoalProto =
    muan::proto::StackProto<WristGoal, 256>;

}  // namespace wrist
}  // namespace c2019

#endif  // C2019_SUBSYSTEMS_SUPERSTRUCTURE_WRIST_QUEUE_TYPES_H_
