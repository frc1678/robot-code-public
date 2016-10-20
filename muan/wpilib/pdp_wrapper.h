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

  void SendValues();

  using PdpMessage = muan::proto::StackProto<PdpStatus, 256>;

  using Queue = muan::queues::MessageQueue<PdpMessage, 100>;
  void SetQueue(Queue* pdp_queue);

 private:
  PowerDistributionPanel pdp_;
  Queue* queue_{nullptr};
};

}  // wpilib

}  // muan

#endif  // MUAN_WPILIB_PDP_WRAPPER_H_
