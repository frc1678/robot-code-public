#include "drivetrain_queues.h"

namespace frc1678 {

namespace drivetrain {

muan::queues::MessageQueue<StackDrivetrainGoal> goal_queue;
muan::queues::MessageQueue<StackDrivetrainInput> input_queue;
muan::queues::MessageQueue<StackDrivetrainStatus> status_queue;
muan::queues::MessageQueue<StackDrivetrainOutput> output_queue;

} /* drivetrain */

} /* frc1678 */
