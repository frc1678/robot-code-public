#ifndef MUAN_WEBDASH_QUEUE_TYPES_H_
#define MUAN_WEBDASH_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "muan/webdash/webdash.pb.h"

namespace muan {

namespace webdash {

using AutoSelectionProto = muan::proto::StackProto<::muan::webdash::AutoSelection, 256>;

using AutoSelectionQueue = muan::queues::MessageQueue<AutoSelectionProto>;

}  // namespace webdash

}  // namespace muan

#endif  // MUAN_WEBDASH_QUEUE_TYPES_H_
