#include "generic_robot/queue_manager/queue_manager.h"

namespace generic_robot {

QueueManager& QueueManager::GetInstance() {
  static QueueManager instance;
  return instance;
}

MessageQueue<muan::proto::StackProto<PdpStatus, 1024>>& QueueManager::pdp_status_queue() {
  return pdp_status_queue_;
}

muan::wpilib::DriverStationQueue& QueueManager::driver_station_queue() { return driver_station_queue_; }

}  // namespace generic_robot
