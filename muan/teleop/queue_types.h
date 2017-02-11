#ifndef C2017_MUAN_TELEOP_QUEUE_TYPES_H_
#define C2017_MUAN_TELEOP_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "c2017/muan/teleop/joystick.pb.h"

namespace c2017 {
namespace joystick {

using XboxJoystickProto = muan::proto::StackProto<XboxJoystickStatus, 512>;
using WheelStatusProto = muan::proto::StackProto<WheelStatus, 512>;
using ThrottleStatusProto = muan::proto::StackProto<ThrottleStatus, 512>;

using XboxJoystickQueue = muan::queues::MessageQueue<XboxJoystickProto, 512>;
using WheelStatusQueue = muan::queues::MessageQueue<WheelStatusProto, 512>;
using ThrottleStatusQueue = muan::queues::MessageQueue<ThrottleStatusProto, 512>;

}  // namespace joystick
}  // namespace c2017

#endif  // C2017_MUAN_TELEOP_QUEUE_TYPES_H_
