#ifndef FRC971_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_H_
#define FRC971_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_H_

#include "third_party/aos/common/controls/polytope.h"

#include "third_party/frc971/shifter_hall_effect.h"
#include "third_party/frc971/control_loops/drivetrain/gear.h"
#include "third_party/frc971/control_loops/drivetrain/queue_types.h"
#include "third_party/frc971/control_loops/state_feedback_loop.h"
#include "third_party/frc971/control_loops/drivetrain/drivetrain_config.h"

namespace frc971 {
namespace control_loops {
namespace drivetrain {

class PolyDrivetrain {
 public:
  PolyDrivetrain(const DrivetrainConfig &dt_config,
                 StateFeedbackLoop<7, 2, 3> *kf);

  int controller_index() const { return loop_->index(); }

  // Computes the speed of the motor given the hall effect position and the
  // speed of the robot.
  double MotorSpeed(const constants::ShifterHallEffect &hall_effect,
                    double shifter_position, double velocity, Gear gear);

  void SetGoal(const ::frc971::control_loops::drivetrain::GoalProto &goal);

  void SetPosition(
      const ::frc971::control_loops::drivetrain::InputProto *position,
      Gear left_gear, Gear right_gear);

  double FilterVelocity(double throttle) const;

  double MaxVelocity();

  void Update();

  void SetOutput(::frc971::control_loops::drivetrain::OutputProto *output);

  void PopulateStatus(::frc971::control_loops::drivetrain::StatusProto *status);

  // Computes the next state of a shifter given the current state and the
  // requested state.
  Gear UpdateSingleGear(Gear requested_gear, Gear current_gear);

 private:
  StateFeedbackLoop<7, 2, 3> *kf_;

  const ::aos::controls::HVPolytope<2, 4, 4> U_Poly_;

  ::std::unique_ptr<StateFeedbackLoop<2, 2, 2>> loop_;

  const double ttrust_;
  double wheel_;
  double throttle_;
  bool quickturn_;

  Gear left_gear_;
  Gear right_gear_;

  ::frc971::control_loops::drivetrain::InputProto last_position_;
  ::frc971::control_loops::drivetrain::InputProto position_;
  int counter_;
  DrivetrainConfig dt_config_;

  double goal_left_velocity_ = 0.0;
  double goal_right_velocity_ = 0.0;

  // Stored from the last iteration, for logging shifting logic.
  double left_motor_speed_ = 0.0;
  double right_motor_speed_ = 0.0;
  double current_left_velocity_ = 0.0;
  double current_right_velocity_ = 0.0;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace frc971

#endif  // FRC971_CONTROL_LOOPS_DRIVETRAIN_POLYDRIVETRAIN_H_
