#include "c2019/subsystems/superstructure/winch/winch.h"
#include "gtest/gtest.h"

namespace c2019 {
namespace winch {

class WinchTest : public ::testing::Test {
 public:
  WinchTest() {}

  void Update(bool outputs_enabled) {
    winch_.SetGoal(winch_goal_proto_);
    winch_.Update(winch_input_proto_, &winch_output_proto_,
                  &winch_status_proto_, outputs_enabled);
  }

  WinchInputProto winch_input_proto_;
  WinchStatusProto winch_status_proto_;
  WinchOutputProto winch_output_proto_;
  WinchGoalProto winch_goal_proto_;

 private:
  Winch winch_;
};

TEST_F(WinchTest, Disabled) {
  winch_goal_proto_->set_climb_goal(BUDDY);
  winch_goal_proto_->set_winch(true);

  Update(false);

  EXPECT_FALSE(winch_output_proto_->drop_forks());
  EXPECT_EQ(winch_output_proto_->winch_voltage(), 0);
}

TEST_F(WinchTest, SoloClimb) {
  winch_goal_proto_->set_climb_goal(SOLO);
  winch_goal_proto_->set_winch(true);

  Update(true);

  EXPECT_FALSE(winch_output_proto_->drop_forks());
  EXPECT_EQ(winch_output_proto_->winch_voltage(), 0);
}

TEST_F(WinchTest, BuddyClimb) {
  winch_goal_proto_->set_climb_goal(BUDDY);
  winch_goal_proto_->set_winch(true);

  Update(true);

  EXPECT_TRUE(winch_output_proto_->drop_forks());
  EXPECT_EQ(winch_output_proto_->winch_voltage(), 12);
}

}  // namespace winch
}  // namespace c2019
