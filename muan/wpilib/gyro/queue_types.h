#ifndef MUAN_WPILIB_GYRO_QUEUE_TYPES_H_
#define MUAN_WPILIB_GYRO_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "muan/wpilib/gyro/gyro.pb.h"

namespace muan {

namespace wpilib {

namespace gyro {

using GyroMessageProto = muan::proto::StackProto<GyroMessage, 256>;
using GyroQueue = muan::queues::MessageQueue<GyroMessageProto>;

}  // namespace gyro

}  // namespace wpilib

}  // namespace muan

#endif  //  MUAN_WPILIB_GYRO_QUEUE_TYPES_H_
