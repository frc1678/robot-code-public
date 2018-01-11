#ifndef MUAN_WPILIB_PDP_WRAPPER_H_
#define MUAN_WPILIB_PDP_WRAPPER_H_

#include "WPILib.h"
#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"
#include "muan/wpilib/queue_types.h"

namespace muan {

namespace wpilib {

class PdpWrapper {
 public:
  explicit PdpWrapper(int module = 0);
  ~PdpWrapper() = default;

  using Queue = muan::queues::MessageQueue<PdpMessage>;
  void SetQueue(Queue* pdp_queue);

 private:
  friend class CanWrapper;

  int module_;

  // Read values from the PDP. This is not realtime and should only be called
  // from the CAN thread. It's private so it can only be called by friend
  // classes, like CanWrapper.
  void SendValues();

  Queue* queue_{nullptr};

  int status_;
  uint64_t num_failures_ = 0;
};

}  // namespace wpilib

}  // namespace muan

#endif  // MUAN_WPILIB_PDP_WRAPPER_H_
