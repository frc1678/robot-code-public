#ifndef MUAN_WPILIB_DS_SENDER_H_
#define MUAN_WPILIB_DS_SENDER_H_

#include "muan/wpilib/queue_types.h"

namespace muan {

namespace wpilib {

class DriverStationSender {
 public:
  explicit DriverStationSender(DriverStationQueue* ds_queue, GameSpecificStringQueue* gss_queue = nullptr);
  void Send();

 private:
  DriverStationQueue* ds_queue_;
  GameSpecificStringQueue* gss_queue_;
};

}  // namespace wpilib

}  // namespace muan

#endif  // MUAN_WPILIB_DS_SENDER_H_
