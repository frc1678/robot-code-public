#ifndef O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_QUEUETYPES_H_
#define O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_QUEUETYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "o2016/subsystems/superstructure/catapult/catapult.pb.h"

namespace o2016 {

namespace catapult{

using CatapultOutputProto = muan::proto::StackProto<CatapultOutput, 1024>;
using CatapultStatusProto = muan::proto::StackProto<CatapultStatus, 1024>;
using CatapultInputProto = muan::proto::StackProto<CatapultInput, 1024>;
using CatapultGoalProto = muan::proto::StackProto<CatapultGoal, 1024>;

using CatapultOutputQueue = muan::queues::MessageQueue<CatapultOutputProto, 100>;
using CatapultStatusQueue = muan::queues::MessageQueue<CatapultStatusProto, 100>;
using CatapultInputQueue = muan::queues::MessageQueue<CatapultInputProto, 100>;
using CatapultGoalQueue = muan::queues::MessageQueue<CatapultGoalProto, 100>;

} // catapult

} // o2016

#endif // O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_QUEUETYPES_H_
