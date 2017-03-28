#ifndef C2017_WEBDASH_QUEUE_TYPES_H_
#define C2017_WEBDASH_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "c2017/webdash/webdash.pb.h"

namespace c2017 {

namespace webdash {

using AutoSelectionProto = muan::proto::StackProto<::c2017::webdash::AutoSelection, 256>;

using AutoSelectionQueue = muan::queues::MessageQueue<AutoSelectionProto, 200>;

}  // namespace webdash

}  // namespace c2017

#endif  // C2017_WEBDASH_QUEUE_TYPES_H_
