#ifndef C2017_SUBSYSTEMS_LIGHTS_QUEUE_TYPES_H_
#define C2017_SUBSYSTEMS_LIGHTS_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "c2017/subsystems/lights/lights.pb.h"

namespace c2017 {
namespace lights {

using LightsOutputProto = muan::proto::StackProto<LightsOutput, 1024>;

using LightsOutputQueue = muan::queues::MessageQueue<LightsOutputProto>;

}  // namespace lights
}  // namespace c2017

#endif  // C2017_SUBSYSTEMS_LIGHTS_QUEUE_TYPES_H_
