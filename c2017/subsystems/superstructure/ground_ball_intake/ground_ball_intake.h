#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_BALL_INTAKE_GROUND_BALL_INTAKE_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_BALL_INTAKE_GROUND_BALL_INTAKE_H_

#include "muan/wpilib/queue_types.h"
#include "c2017/subsystems/superstructure/ground_ball_intake/queue_types.h"

namespace c2017 {

namespace ground_ball_intake {

class GroundBallIntake {
 public:
  GroundBallIntake() = default;
  GroundBallIntakeOutputProto Update(const DriverStationStatus& robot_state, GroundBallIntakeGoalProto goal_);
  GroundBallIntakeStatusProto get_status();
 private:
  GroundBallIntakeOutputProto output_;
  GroundBallIntakeStatusProto status_;
  GroundBallIntakeGoalProto goal_;
};

} //ground_ball_intake

} //c2017

#endif //C2017_SUBSYSTEMS_SUPERSTRUCTURE_GROUND_BALL_INTAKE_GROUND_BALL_INTAKE_H_
