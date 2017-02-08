#ifndef MUAN_WPILIB_PDP_WRAPPER_H_
#define MUAN_WPILIB_PDP_WRAPPER_H_

#include "WPILib.h"
#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "muan/wpilib/state.pb.h"

namespace muan {

namespace wpilib {

class PdpWrapper {
 public:
  PdpWrapper() = default;
  ~PdpWrapper() = default;

  using PdpMessage = muan::proto::StackProto<PdpStatus, 1024>;

  using Queue = muan::queues::MessageQueue<PdpMessage, 100>;
  void SetQueue(Queue* pdp_queue);

 private:
  friend class CanWrapper;

  // Read values from the PDP. This is not realtime and should only be called
  // from the CAN thread. It's private so it can only be called by friend
  // classes, like CanWrapper.
  void SendValues();

  PowerDistributionPanel pdp_;
  Queue* queue_{nullptr};
};

}  // namespace wpilib

}  // namespace muan

#endif  // MUAN_WPILIB_PDP_WRAPPER_H_
