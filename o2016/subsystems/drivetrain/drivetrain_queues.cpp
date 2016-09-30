#include "drivetrain_queues.h"

namespace frc1678 {

namespace drivetrain {

muan::queues::MessageQueue<DrivetrainGoal> goal_queue;
muan::queues::MessageQueue<DrivetrainInput> input_queue;
muan::queues::MessageQueue<DrivetrainStatus> status_queue;
muan::queues::MessageQueue<DrivetrainOutput> output_queue;

} /* drivetrain */

} /* frc1678 */
