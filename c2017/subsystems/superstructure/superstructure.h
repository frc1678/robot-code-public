#ifndef C2017_SUBSYSTEMS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define C2017_SUBSYSTEMS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include "c2017/subsystems/superstructure/shooter/queue_types.h"
#include "c2017/subsystems/superstructure/magazine/queue_types.h"
#include "c2017/subsystems/superstructure/ground_gear_intake/queue_types.h"
#include "c2017/subsystems/superstructure/ground_ball_intake/queue_types.h"
#include "c2017/subsystems/superstructure/climber/queue_types.h"
#include "c2017/subsystems/superstructure/queue_types.h"
#include "c2017/queue_manager/queue_manager.h"
#include "c2017/wpilib/queue_types.h"
#include "c2017/subsystems/superstructure/ground_gear_intake/ground_gear_intake.h"
#include "c2017/subsystems/superstructure/ground_ball_intake/ground_ball_intake.h"
#include "c2017/subsystems/superstructure/shooter/shooter_controller.h"
#include "c2017/subsystems/superstructure/magazine/magazine.h"
#include "c2017/subsystems/superstructure/climber/climber.h"
#include "muan/wpilib/queue_types.h"

namespace c2017 {
namespace superstructure {

class SuperStructure {
 public:
  SuperStructure();

  void Update();

 private:
  c2017::shooter::ShooterController shooter_;
  c2017::ground_gear_intake::GroundGearIntake ground_gear_intake_;
  c2017::ground_ball_intake::GroundBallIntake ground_ball_intake_;
  c2017::magazine::Magazine magazine_;
  c2017::climber::Climber climber_;

  c2017::magazine::MagazineGoalProto magazine_goal_;
  c2017::shooter::ShooterGoalProto shooter_goal_;
  c2017::climber::ClimberGoalProto climber_goal_;

  c2017::superstructure::SuperstructureStatusProto superstructure_status_proto_;

  bool is_shooting_ = false;

  void UpdateShooter();
  void Shoot(const c2017::shooter::ShooterStatusProto& s_s);
  void Spinup(const c2017::shooter_group::ShooterGroupGoalProto& s_g);
  void UpdateIntake();
  void SetWpilibOutput();
};

}  // namespace superstructure
}  // namespace c2017

#endif  // C2017_SUBSYSTEMS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
