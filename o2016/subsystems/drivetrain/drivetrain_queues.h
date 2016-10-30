#ifndef O2016_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_QUEUES_H_
#define O2016_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_QUEUES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "o2016/subsystems/drivetrain/drivetrain.pb.h"

namespace frc1678 {

namespace drivetrain {

using StackDrivetrainGoal = muan::proto::StackProto<DrivetrainGoal, 128>;
using StackDrivetrainInput = muan::proto::StackProto<DrivetrainInput, 100>;
using StackDrivetrainStatus = muan::proto::StackProto<DrivetrainStatus, 128>;
using StackDrivetrainOutput = muan::proto::StackProto<DrivetrainOutput, 100>;

extern muan::queues::MessageQueue<StackDrivetrainGoal> goal_queue;
extern muan::queues::MessageQueue<StackDrivetrainInput> input_queue;
extern muan::queues::MessageQueue<StackDrivetrainStatus> status_queue;
extern muan::queues::MessageQueue<StackDrivetrainOutput> output_queue;

} /* drivetrain */

} /* frc1678 */

#endif  // O2016_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_QUEUES_H_
