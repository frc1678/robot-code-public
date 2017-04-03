#ifndef MUAN_WPILIB_DS_SENDER_H_
#define MUAN_WPILIB_DS_SENDER_H_

#include "muan/wpilib/queue_types.h"

namespace muan {

namespace wpilib {

class DriverStationSender {
 public:
  explicit DriverStationSender(DriverStationQueue* ds_queue);
  void Send();

 private:
  DriverStationQueue* queue_;
};

}  // namespace wpilib

}  // namespace muan

#endif  // MUAN_WPILIB_DS_SENDER_H_
