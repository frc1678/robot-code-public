#ifndef MUAN_LIME_LIME_H_
#define MUAN_LIME_LIME_H_

#include <cmath>
#include "muan/lime/queue_types.h"
#include "muan/logging/logger.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/queue_types.h"
#include "third_party/wpilibsuite/ntcore/src/main/native/include/networktables/NetworkTable.h"
#include "third_party/wpilibsuite/ntcore/src/main/native/include/networktables/NetworkTableEntry.h"
#include "third_party/wpilibsuite/ntcore/src/main/native/include/networktables/NetworkTableInstance.h"

namespace muan {
namespace lime {

class Lime {
 public:
  Lime(double lime_height, double lime_angle, double object_height);
  Lime(double lime_height, double lime_angle, double object_height,
       double dist_factor, double dist_offset);
  void GetTable();
  void Update();
  double ObjectDistance(double vertical_angle);

 private:
  LimeStatusQueue* status_queue_;
  LimeGoalQueue::QueueReader goal_reader_{
      muan::queues::QueueManager<LimeGoalProto>::Fetch()->MakeReader()};
  FRIEND_TEST(LimeTest, HasNoTarget);
  double lime_height_;
  double lime_angle_;
  double object_height_;
  double dist_factor_ = 1;
  double dist_offset_ = 0;
  double distance_;
  double target_vertical_angle_;
  double target_horizontal_angle_;
  double target_area_;
  double target_skew_;
  bool target_present_;
};

}  // namespace lime
}  // namespace muan

#endif  // MUAN_LIME_LIME_H_
