#ifndef MUAN_VISION_QUEUE_TYPES_H_
#define MUAN_VISION_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "muan/vision/vision.pb.h"

namespace muan {
namespace vision {

using VisionPositionProto = muan::proto::StackProto<VisionPosition, 256>;
using VisionPositionQueue = muan::queues::MessageQueue<VisionPositionProto, 100>;

}  // namespace vision
}  // namespace muan

#endif  // MUAN_VISION_QUEUE_TYPES_H_
