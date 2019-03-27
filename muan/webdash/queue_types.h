#ifndef MUAN_WEBDASH_QUEUE_TYPES_H_
#define MUAN_WEBDASH_QUEUE_TYPES_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "muan/webdash/webdash.pb.h"

namespace muan {
namespace webdash {

using WebdashProto =
    muan::proto::StackProto<Webdash, 512>;
using AutoProto =
    muan::proto::StackProto<Auto, 512>;

using WebdashQueue = muan::queues::MessageQueue<WebdashProto>;
using AutoQueue = muan::queues::MessageQueue<AutoProto>;

}  // namespace webdash
}  // namespace muan

#endif  // MUAN_WEBDASH_QUEUE_TYPES_H_
