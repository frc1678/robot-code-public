#ifndef C2017_VISION_QUEUE_TYPES_H_
#define C2017_VISION_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "c2017/vision/vision.pb.h"

namespace c2017 {
namespace vision {

using VisionInputProto = muan::proto::StackProto<VisionInput, 256>;
using VisionStatusProto = muan::proto::StackProto<VisionStatus, 256>;
using VisionGoalProto = muan::proto::StackProto<VisionGoal, 256>;

using VisionInputQueue = muan::queues::MessageQueue<VisionInputProto, 100>;
using VisionStatusQueue = muan::queues::MessageQueue<VisionStatusProto, 100>;
using VisionGoalQueue = muan::queues::MessageQueue<VisionGoalProto, 100>;

}  // namespace vision
}  // namespace c2017

#endif  // C2017_VISION_QUEUE_TYPES_H_
