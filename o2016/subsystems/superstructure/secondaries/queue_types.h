#ifndef O2016_SUBSYSTEMS_SUPERSTRUCTURE_SECONDARIES_QUEUE_TYPES_H_
#define O2016_SUBSYSTEMS_SUPERSTRUCTURE_SECONDARIES_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "o2016/subsystems/superstructure/secondaries/secondaries.pb.h"

namespace o2016 {
namespace secondaries {

using SecondariesGoalProto = muan::proto::StackProto<::o2016::secondaries::SecondariesGoal, 1024>;
using SecondariesOutputProto = muan::proto::StackProto<::o2016::secondaries::SecondariesOutput, 1024>;

using SecondariesOutputQueue = muan::queues::MessageQueue<SecondariesOutputProto, 200>;

}  // secondaries
}  // o2016

#endif  // O2016_SUBSYSTEMS_SUPERSTRUCTURE_SECONDARIES_QUEUE_TYPES_H_
