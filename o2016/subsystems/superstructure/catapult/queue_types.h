#ifndef O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_QUEUETYPES_H_
#define O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_QUEUETYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "o2016/subsystems/superstructure/catapult/catapult.pb.h"

using CatapultOutputProto = muan::proto::StackProto<CatapultOutput, 100>;
using CatapultStatusProto = muan::proto::StackProto<CatapultStatus, 100>;
using CatapultInputProto = muan::proto::StackProto<CatapultInput, 100>;
using CatapultGoalProto = muan::proto::StackProto<CatapultGoal, 100>;

using CatapultOutputQueue = muan::queues::MessageQueue<CatapultOutputProto, 100>;
using CatapultStatusQueue = muan::queues::MessageQueue<CatapultStatusProto, 100>;
using CatapultInputQueue = muan::queues::MessageQueue<CatapultInputProto, 100>;
using CatapultGoalQueue = muan::queues::MessageQueue<CatapultGoalProto, 100>;

#endif // O2016_SUBSYSTEMS_SUPERSTRUCTURE_CATAPULT_QUEUETYPES_H_
