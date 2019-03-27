#ifndef C2019_SUBSYSTEMS_LIMELIGHT_LIMELIGHT_H_
#define C2019_SUBSYSTEMS_LIMELIGHT_LIMELIGHT_H_

#include <cmath>
#include "c2019/subsystems/limelight/queue_types.h"
#include "muan/logging/logger.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/queue_types.h"
#include "networktables/NetworkTable.h"
#include "networktables/NetworkTableEntry.h"
#include "networktables/NetworkTableInstance.h"

namespace c2019 {
namespace limelight {

class Limelight {
 public:
  Limelight(double limelight_height, double limelight_angle,
            double object_height);
  ~Limelight() = default;
  void operator()();
  void Update();
  void BackUpdate();

 private:
  LimelightStatusQueue* status_queue_;
  LimelightGoalQueue::QueueReader goal_reader_{
      muan::queues::QueueManager<LimelightGoalProto>::Fetch()->MakeReader()};
  // FRIEND_TEST(LimelightTest, HasNoTarget);
  double target_dist_ = 0;
  double horiz_angle_ = 0;
  double limelight_height_;
  double limelight_angle_;
  double object_height_;
  double heading_;
  bool to_the_left_;
  double pricey_horiz_angle_;
  double pricey_target_dist_;
  double prev_latency_ = 0;
  double bottom_prev_latency_ = 0;

  std::atomic<bool> running_;
  int bad_ticks_ = 0;
  int bottom_bad_ticks_ = 0;
};
}  // namespace limelight
}  // namespace c2019

#endif  // C2019_SUBSYSTEMS_LIMELIGHT_LIMELIGHT_H_
