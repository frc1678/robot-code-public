#ifndef C2019_SUBSYSTEMS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
#define C2019_SUBSYSTEMS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_

#include "c2019/subsystems/superstructure/cargo_intake/cargo_intake.h"
#include "c2019/subsystems/superstructure/cargo_intake/queue_types.h"
#include "c2019/subsystems/superstructure/elevator/elevator.h"
#include "c2019/subsystems/superstructure/elevator/queue_types.h"
#include "c2019/subsystems/superstructure/ground_hatch_intake/ground_hatch_intake.h"
#include "c2019/subsystems/superstructure/ground_hatch_intake/queue_types.h"
#include "c2019/subsystems/superstructure/hatch_intake/hatch_intake.h"
#include "c2019/subsystems/superstructure/hatch_intake/queue_types.h"
#include "c2019/subsystems/superstructure/queue_types.h"
#include "c2019/subsystems/superstructure/winch/queue_types.h"
#include "c2019/subsystems/superstructure/winch/winch.h"
#include "c2019/subsystems/superstructure/wrist/queue_types.h"
#include "c2019/subsystems/superstructure/wrist/wrist.h"
#include "muan/queues/queue_manager.h"
#include "muan/wpilib/queue_types.h"

namespace c2019 {
namespace superstructure {

// TODO(hanson) test these numbers
// elevator constants
constexpr double kHatchShipForwardsHeight = .4;
constexpr double kHatchShipBackwardsHeight = 0.;
constexpr double kHatchRocketFirstHeight = .4;
constexpr double kHatchRocketBackwardsHeight = 0.;
constexpr double kHatchRocketSecondHeight = 1.2;
constexpr double kHatchRocketThirdHeight = 2.;
constexpr double kHatchLoadingStationHeight = .4;
constexpr double kHatchGroundHeight = 0.;
constexpr double kCargoShipForwardsHeight = 1.;
constexpr double kCargoShipBackwardsHeight = 0;
constexpr double kCargoRocketFirstHeight = .6;
constexpr double kCargoRocketBackwardsHeight = 0.;
constexpr double kCargoRocketSecondHeight = 1.4;
constexpr double kCargoRocketThirdHeight = 2.;
constexpr double kCargoGroundHeight = 0.;
constexpr double kHandoffHeight = 0.;
constexpr double kSpitHeight = 0.;
constexpr double kStowHeight = 0.;
constexpr double kClimbHeight = 0.;
constexpr double kElevatorSafeHeight = 0.05;
constexpr double kElevatorMinHeight = 0.;
constexpr double kElevatorMaxHeight = 2.;
constexpr double kElevatorHandoffTolerance = 2e-3;

// wrist constants
constexpr double kHatchForwardsAngle = 0.;
constexpr double kHatchBackwardsAngle = 3.1;
constexpr double kCargoRocketFirstAngle = 0.;
constexpr double kCargoRocketSecondAngle = 0.;
constexpr double kCargoRocketThirdAngle = M_PI / 12.;
constexpr double kCargoRocketBackwardsAngle = M_PI * (5. / 6.);
constexpr double kCargoShipForwardsAngle = 0.;
constexpr double kCargoShipBackwardsAngle = M_PI * (5. / 6.);
constexpr double kCargoGroundAngle = 0.;
constexpr double kHandoffAngle = 3.1;
constexpr double kStowAngle = M_PI / 2.;
constexpr double kClimbAngle = 0.;
constexpr double kWristSafeAngle = 1.4;
constexpr double kWristMinAngle = 0.;
constexpr double kWristMaxAngle = M_PI;
constexpr double kWristHandoffTolerance = 3. * (M_PI / 180.);

class Superstructure {
 public:
  Superstructure();

  void Update();

 private:
  void SetGoal(const SuperstructureGoalProto& goal);
  void GoToState(SuperstructureState state = SuperstructureState::IDLE,
                 IntakeGoal intake = IntakeGoal::INTAKE_NONE);
  void RunStateMachine();
  void BoundGoal(double* elevator_goal, double* wrist_goal);

  elevator::ElevatorGoalProto PopulateElevatorGoal();
  wrist::WristGoalProto PopulateWristGoal();
  ground_hatch_intake::GroundHatchIntakeGoalProto
  PopulateGroundHatchIntakeGoal();
  hatch_intake::HatchIntakeGoalProto PopulateHatchIntakeGoal();
  cargo_intake::CargoIntakeGoalProto PopulateCargoIntakeGoal();
  winch::WinchGoalProto PopulateWinchGoal();

  c2019::cargo_intake::CargoIntake cargo_intake_;
  c2019::elevator::Elevator elevator_;
  c2019::ground_hatch_intake::GroundHatchIntake ground_hatch_intake_;
  c2019::hatch_intake::HatchIntake hatch_intake_;
  c2019::wrist::Wrist wrist_;
  c2019::winch::Winch winch_;

  cargo_intake::CargoIntakeStatusProto cargo_intake_status_;
  elevator::ElevatorStatusProto elevator_status_;
  ground_hatch_intake::GroundHatchIntakeStatusProto ground_hatch_intake_status_;
  hatch_intake::HatchIntakeStatusProto hatch_intake_status_;
  wrist::WristStatusProto wrist_status_;
  winch::WinchStatusProto winch_status_;

  SuperstructureStatusProto status_;

  SuperstructureGoalQueue::QueueReader goal_reader_;
  SuperstructureInputQueue::QueueReader input_reader_;
  SuperstructureStatusQueue* status_queue_;
  SuperstructureOutputQueue* output_queue_;

  muan::wpilib::DriverStationQueue::QueueReader ds_status_reader_;

  double elevator_height_;
  double wrist_angle_;

  bool crawling_ = false;
  bool high_gear_ = true;
  bool crawler_down_ = false;
  bool brake_ = false;

  bool should_climb_ = false;
  bool buddy_ = false;

  SuperstructureState state_ = SuperstructureState::CALIBRATING;
  IntakeGoal intake_goal_ = IntakeGoal::INTAKE_NONE;
};

}  // namespace superstructure
}  // namespace c2019

#endif  // C2019_SUBSYSTEMS_SUPERSTRUCTURE_SUPERSTRUCTURE_H_
