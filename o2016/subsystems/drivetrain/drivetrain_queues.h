#ifndef O2016_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_QUEUES_H_
#define O2016_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_QUEUES_H_

#include "muan/queues/message_queue.h"
#include "o2016/subsystems/drivetrain/drivetrain.pb.h"

namespace frc1678 {

namespace drivetrain {

extern muan::queues::MessageQueue<DrivetrainGoal> goal_queue;
extern muan::queues::MessageQueue<DrivetrainInput> input_queue;
extern muan::queues::MessageQueue<DrivetrainStatus> status_queue;
extern muan::queues::MessageQueue<DrivetrainOutput> output_queue;

} /* drivetrain */

} /* frc1678 */

#endif  // O2016_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_QUEUES_H_
