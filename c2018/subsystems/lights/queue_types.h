#ifndef C2018_SUBSYSTEMS_LIGHTS_QUEUE_TYPES_H_
#define C2018_SUBSYSTEMS_LIGHTS_QUEUE_TYPES_H_

#include "c2018/subsystems/lights/lights.pb.h"
#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"

namespace c2018 {
namespace lights {

using LightsOutputProto = muan::proto::StackProto<LightsOutput, 1024>;

using LightsOutputQueue = muan::queues::MessageQueue<LightsOutputProto>;

}  // namespace lights
}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_LIGHTS_QUEUE_TYPES_H_

