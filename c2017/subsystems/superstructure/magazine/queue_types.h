#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_MAGAZINE_QUEUE_TYPES_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_MAGAZINE_QUEUE_TYPES_H_

#include "c2017/subsystems/superstructure/magazine/magazine.pb.h"
#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"

namespace c2017 {
namespace magazine {

using MagazineGoalProto =
    muan::proto::StackProto<::c2017::magazine::MagazineGoal, 256>;
using MagazineOutputProto =
    muan::proto::StackProto<::c2017::magazine::MagazineOutput, 256>;
using MagazineStatusProto =
    muan::proto::StackProto<::c2017::magazine::MagazineStatus, 256>;

using MagazineGoalQueue = muan::queues::MessageQueue<MagazineGoalProto>;
using MagazineOutputQueue = muan::queues::MessageQueue<MagazineOutputProto>;
using MagazineStatusQueue = muan::queues::MessageQueue<MagazineStatusProto>;

}  // namespace magazine
}  // namespace c2017

#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_MAGAZINE_QUEUE_TYPES_H_
