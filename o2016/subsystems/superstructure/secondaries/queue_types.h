#ifndef O2016_SUBSYSTEMS_SUPERSTRUCTURE_SECONDARIES_QUEUETYPES_H_
#define O2016_SUBSYSTEMS_SUPERSTRUCTURE_SECONDARIES_QUEUETYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "o2016/subsystems/superstructure/secondaries/secondaries.pb.h"

namespace o2016 {

namespace secondaries {

using SecondariesGoalProto = muan::proto::StackProto<::o2016::secondaries::SecondariesGoal, 1024>;
using SecondariesOutputProto = muan::proto::StackProto<::o2016::secondaries::SecondariesOutput, 1024>;

using SecondariesGoalQueue = muan::queues::MessageQueue<SecondariesGoalProto, 200>;
using SecondariesOutputQueue = muan::queues::MessageQueue<SecondariesOutputProto, 200>;
}
}

#endif  // O2016_SUBSYSTEMS_SUPERSTRUCTURE_SECONDARIES_QUEUETYPES_H_
