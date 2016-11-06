#include "catapult.h"
#include "gtest/gtest.h"
#include "o2016/subsystems/superstructure/catapult/scoop/scoop_constants.h"
#include "o2016/subsystems/superstructure/catapult/stop/stop_constants.h"
#include "muan/control/state_space_plant.h"
#include "muan/control/state_space_observer.h"
#include <iostream>

using namespace muan::units;
using namespace o2016::catapult;

class CatapultTest : public ::testing::Test {
 public:
  CatapultTest() :
    scoop_plant_(muan::control::StateSpacePlant<1, 2, 1>(frc1678::scoop::controller::A(), frc1678::scoop::controller::B(), frc1678::scoop::controller::C())),
    stop_plant_(muan::control::StateSpacePlant<1, 2, 1>(frc1678::stop::controller::A(), frc1678::stop::controller::B(), frc1678::stop::controller::C())) {}

  void UpdateTest(Length distance_to_target) {
    input_->set_scoop_pot(scoop_plant_.y(0));
    input_->set_hardstop_pot(stop_plant_.y(0));
    // input_.set_distance_to_target(distance_to_target);

    catapult_.Update(input_, goal_);

    status_ = catapult_.status();
    output_ = catapult_.output();

    scoop_plant_.Update((Eigen::Matrix<double, 1, 1>() << output_->scoop_output()).finished());
    stop_plant_.Update((Eigen::Matrix<double, 1, 1>() << output_->hardstop_output()).finished());
  }

  CatapultOutputProto output_;
  CatapultStatusProto status_;
  CatapultInputProto input_;
  CatapultGoalProto goal_;

 protected:
  Catapult catapult_;
  muan::control::StateSpacePlant<1, 2, 1> scoop_plant_;
  muan::control::StateSpacePlant<1, 2, 1> stop_plant_;
};

// Move it a bunch and make sure it finished all the commands
TEST_F(CatapultTest, Terminates) {
  status_->set_scoop_angle(0 * rad);
  status_->set_hardstop_angle(0 * rad);
  for(int i = 0; i < 3; i++) {
    goal_->set_goal(CatapultGoal::INTAKE);
    for(int i = 0; i < 400; i++) {
      UpdateTest(0 * m);
    }
    EXPECT_TRUE(status_->at_goal());
    goal_->set_goal(CatapultGoal::PREP_SHOT);
    for(int i = 0; i < 400; i++) {
      UpdateTest(0 * m);
    }
    // Use assert because the next test will be an
    // infinite loop if this doesn't pass
    EXPECT_TRUE(status_->at_goal());
  }
}

// Go intake -> shoot -> intake, make sure it fills in the transitions
// and gets to each state.
TEST_F(CatapultTest, ValidTransitions) {
  status_->set_scoop_angle(0 * rad);
  status_->set_hardstop_angle(0 * rad);
  // First, make sure it gets to intake position
  goal_->set_goal(CatapultGoal::INTAKE);
  do {
    UpdateTest(0 * m);
    EXPECT_EQ(status_->state(), CatapultStatus::INTAKING);
  } while(!status_->at_goal());
  // Telling it to shoot should make it go to prep_shot
  goal_->set_goal(CatapultGoal::SHOOT);
  UpdateTest(0 * m);
  do {
    EXPECT_EQ(status_->state(), CatapultStatus::PREPING_SHOT);
    UpdateTest(0 * m);
  } while(!status_->at_goal());
  // After prep_shot finishes, it should shoot since we are telling it to
  UpdateTest(0 * m);
  EXPECT_EQ(status_->state(), CatapultStatus::SHOOTING);
  // Telling it to stop shooting should not have an effect while the
  // catapult is extending
  goal_->set_goal(CatapultGoal::INTAKE);
  UpdateTest(0 * m);
  EXPECT_EQ(status_->state(), CatapultStatus::SHOOTING);
  // After it finishes it's shot it should go where we told it
  for(int i = 0; i < 500; i++) {
    UpdateTest(0 * m);
  }
  EXPECT_EQ(status_->state(), CatapultStatus::INTAKING);
}
