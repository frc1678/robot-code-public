#ifndef C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_ELEVATOR_ELEVATOR_CONTROLLER_H_
#define C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_ELEVATOR_ELEVATOR_CONTROLLER_H_

#include "c2018/subsystems/score_subsystem/elevator/elevator_constants.h"
#include "c2018/subsystems/score_subsystem/queue_types.h"
#include "muan/control/calibration/hall_calibration.h"
#include "muan/control/state_space_controller.h"
#include "muan/control/state_space_observer.h"
#include "muan/control/state_space_plant.h"
#include "muan/queues/queue_manager.h"
#include "third_party/aos/common/util/trapezoid_profile.h"

namespace c2018 {

namespace score_subsystem {

namespace elevator {

constexpr double kElevatorAcceleration = 2.3;
constexpr double kElevatorVelocity = 2.2;
constexpr double kElevatorMaxHeight = 2.06;
constexpr double kHallEffectHeight = 0.898;
constexpr double kCalibrationVoltage = 6.0;
constexpr double kElevatorMaxVoltage = 12;

class ElevatorController {
 public:
  ElevatorController();
  void Update(const ScoreSubsystemInputProto& input, ScoreSubsystemOutputProto* output,
              ScoreSubsystemStatusProto* status, bool outputs_enabled);
  Eigen::Matrix<double, 2, 1> UpdateProfiledGoal(double unprofiled_goal_, bool outputs_enabled);
  void SetGoal(double goal);
  double CapU(double elevator_u);

 private:
  muan::control::StateSpacePlant<1, 3, 1> plant_;
  muan::control::StateSpaceController<1, 3, 1> elevator_controller_;
  muan::control::StateSpaceObserver<1, 3, 1> elevator_observer_;

  muan::control::HallCalibration hall_calib_{kHallEffectHeight};

  void SetWeights(bool second_stage, bool has_cube);

  Eigen::Matrix<double, 2, 1> profiled_goal_;
  double unprofiled_goal_;

  aos::util::TrapezoidProfile trapezoid_profile_{std::chrono::milliseconds(5)};

  bool encoder_fault_detected_ = false;
  int num_encoder_fault_ticks_ = 0;

  double old_pos_;
};

constexpr double kEncoderFaultTicksAllowed = 100;

}  // namespace elevator

}  // namespace score_subsystem

}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_ELEVATOR_ELEVATOR_CONTROLLER_H_
