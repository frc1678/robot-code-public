#ifndef O2016_QUEUEMANAGER_QUEUEMANAGER_h_
#define O2016_QUEUEMANAGER_QUEUEMANAGER_h_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"

#include "muan/wpilib/state.pb.h"

using muan::queues::MessageQueue;

namespace o2016 {

// A class that contains all of the queues, and allows anyone to get a reference
// to any queue. This is to avoid having all the queues as global variables
// (because that would be gross). Instead, we can just have an instance of this
// as a global, which is much less sketchy.
class QueueManager {
  public:
    static QueueManager& GetInstance();

    // Note: This needs to be the same as the actual message queue in the
    // PdpWrapper class. If you change that, you will need to change this.
    // It is like this to avoid making QueueManager rely on WPILib.
    MessageQueue<muan::proto::StackProto<PdpStatus, 512>>& get_pdp_status_queue();

  private:
    QueueManager() = default;
    ~QueueManager() = default;

    MessageQueue<muan::proto::StackProto<PdpStatus, 512>> pdp_status_queue_;
};

}

#endif
