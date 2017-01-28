#include "c2017/vision/queue_types.h"
#include "gtest/gtest.h"

TEST(Types, CanConstruct) {
  c2017::vision::VisionInputProto input;
  c2017::vision::VisionStatusProto status;
  c2017::vision::VisionGoalProto goal;

  c2017::vision::VisionInputQueue input_queue;
  c2017::vision::VisionStatusQueue status_queue;
  c2017::vision::VisionGoalQueue goal_queue;
}
