#ifndef GENERIC_ROBOT_QUEUE_MANAGER_QUEUE_MANAGER_H_
#define GENERIC_ROBOT_QUEUE_MANAGER_QUEUE_MANAGER_H_

#include "muan/proto/stack_proto.h"
#include "muan/queues/message_queue.h"

#include "muan/wpilib/queue_types.h"

using muan::queues::MessageQueue;

namespace generic_robot {

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
  MessageQueue<muan::proto::StackProto<PdpStatus, 512>>& pdp_status_queue();
  muan::wpilib::DriverStationQueue& driver_station_queue();

 private:
  QueueManager() = default;
  ~QueueManager() = default;

  MessageQueue<muan::proto::StackProto<PdpStatus, 512>> pdp_status_queue_;
  muan::wpilib::DriverStationQueue driver_station_queue_;
};

}  // namespace generic_robot

#endif  // GENERIC_ROBOT_QUEUE_MANAGER_QUEUE_MANAGER_H_
