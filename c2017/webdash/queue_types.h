#ifndef C2017_WEBDASH_QUEUE_TYPES_H_
#define C2017_WEBDASH_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "c2017/webdash/webdash.pb.h"

namespace c2017 {

namespace webdash {

using WebDashProto = muan::proto::StackProto<::c2017::webdash::WebDash, 256>;

using WebDashQueue = muan::queues::MessageQueue<WebDashProto, 200>;

}  // namespace webdash

}  // namespace c2017

#endif  // C2017_WEBDASH_QUEUE_TYPES_H_
