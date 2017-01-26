#include "o2016/subsystems/superstructure/catapult/queue_types.h"
#include "gtest/gtest.h"

// Make sure enough memory has been allocated
TEST(Types, CanConstruct) {
  o2016::catapult::CatapultOutputProto output;
  o2016::catapult::CatapultStatusProto status;
  o2016::catapult::CatapultInputProto input;
  o2016::catapult::CatapultGoalProto goal;

  o2016::catapult::CatapultOutputQueue output_queue;
  o2016::catapult::CatapultStatusQueue status_queue;
  o2016::catapult::CatapultInputQueue input_queue;
  o2016::catapult::CatapultGoalQueue goal_queue;
}
