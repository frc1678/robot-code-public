#ifndef O2016_SUBSYSTEMS_DRIVETRAIN_BUILD_H_
#define O2016_SUBSYSTEMS_DRIVETRAIN_BUILD_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "o2016/subsystems/drivetrain/drivetrain.pb.h"

namespace o2016 {

namespace drivetrain {

using DrivetrainInputProto = muan::proto::StackProto<DrivetrainInput, 80>;
using DrivetrainGoalProto = muan::proto::StackProto<DrivetrainGoal, 210>;
using DrivetrainStatusProto = muan::proto::StackProto<DrivetrainStatus, 300>;
using DrivetrainOutputProto = muan::proto::StackProto<DrivetrainOutput, 80>;

using DrivetrainInputQueue =
    muan::queues::MessageQueue<DrivetrainInputProto, 200>;
using DrivetrainGoalQueue =
    muan::queues::MessageQueue<DrivetrainGoalProto, 200>;
using DrivetrainStatusQueue =
    muan::queues::MessageQueue<DrivetrainStatusProto, 200>;
using DrivetrainOutputQueue =
    muan::queues::MessageQueue<DrivetrainOutputProto, 200>;

}  // drivetrain

}  // o2016

#endif
