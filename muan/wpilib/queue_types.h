#ifndef MUAN_WPILIB_QUEUE_TYPES_H_
#define MUAN_WPILIB_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "muan/wpilib/state.pb.h"

namespace muan {

namespace wpilib {

using DriverStationProto = muan::proto::StackProto<DriverStationStatus, 256>;
using DriverStationQueue = muan::queues::MessageQueue<DriverStationProto>;

using PdpMessage = muan::proto::StackProto<PdpStatus, 512>;
using PdpQueue = muan::queues::MessageQueue<PdpMessage>;

}  // namespace wpilib

}  // namespace muan

#endif  // MUAN_WPILIB_QUEUE_TYPES_H_
