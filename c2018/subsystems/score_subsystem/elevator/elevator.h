#ifndef C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_ELEVATOR_ELEVATOR_H_
#define C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_ELEVATOR_ELEVATOR_H_

#include <cmath>
#include <limits>

#include "c2018/subsystems/score_subsystem/elevator/elevator_constants.h"
#include "c2018/subsystems/score_subsystem/queue_types.h"
#include "muan/control/calibration/hall_calibration.h"
#include "muan/control/state_space_controller.h"
#include "muan/control/state_space_observer.h"
#include "muan/control/state_space_plant.h"
#include "muan/control/trapezoidal_motion_profile.h"
#include "muan/logging/logger.h"
#include "muan/queues/queue_manager.h"
#include "muan/utils/math_utils.h"
namespace c2018 {

namespace score_subsystem {

namespace elevator {

// Trapezoid Profile parameter
constexpr double kElevatorMaxAcceleration = 4.0 * muan::units::mps2;
constexpr double kElevatorMaxVelocity = 2.5 * muan::units::mps;
constexpr muan::control::MotionProfileConstraints kElevatorConstraints = {
    .max_velocity = kElevatorMaxVelocity,
    .max_acceleration = kElevatorMaxAcceleration};

// Capping stuff so it doesn't go boom
constexpr double kElevatorMinHeight = 0.0;
constexpr double kElevatorMaxHeight = 1.92;

// Realistic voltage (SKY)
constexpr double kElevatorMaxVoltage = 12;

// Calibration parameters so it thinks it is where it actually is
constexpr double kHallEffectHeight = 0.92;
constexpr double kCalibrationVoltage = 6;

// Encoder fault stuff so it doesn't get too sad when they break
constexpr double kEncoderFaultMinVoltage = 6;
constexpr double kEncoderFaultTicksAllowed = 100;

class ElevatorController {
 public:
  ElevatorController();  // Sets up the controller, observer, plant, and profile
  void Update(const ScoreSubsystemInputProto& input,
              ScoreSubsystemOutputProto* output,
              ScoreSubsystemStatusProto* status,
              bool outputs_enabled);  // Figures out what the elevator should do
                                      // and what it's doing based on the
                                      // outside data
  void SetGoal(double height);  // Setter for unprofiled_goal_ that also caps it
                                // to kElevatorMin and Max Height

  muan::units::Time TimeLeftUntil(muan::units::Length target,
                                  muan::units::Length final_goal);

  bool is_calibrated() const;  // Getter for if it's calibrated

 private:
  muan::control::MotionProfilePosition unprofiled_goal_;
  muan::control::MotionProfilePosition profiled_goal_;

  muan::control::MotionProfilePosition UpdateProfiledGoal(
      bool outputs_enabled);       // Utilizes the trapezoidal motion profile
  double CapU(double elevator_u);  // Voltage capper to +/- 12

  // State Space magic lol
  muan::control::StateSpacePlant<1, 3, 1> plant_;
  muan::control::StateSpaceController<1, 3, 1> controller_;
  muan::control::StateSpaceObserver<1, 3, 1> observer_;

  // Hall Calibration magic (less magical)
  muan::control::HallCalibration hall_calibration_{kHallEffectHeight};

  // Gain scheduling magic (even less magical)
  void SetWeights(bool second_stage, bool has_cube);

  // Encoder fault stuff
  bool encoder_fault_detected_ = false;
  int num_encoder_fault_ticks_ = 0;
  double old_pos_;
};

}  // namespace elevator

}  // namespace score_subsystem

}  // namespace c2018

#endif  // C2018_SUBSYSTEMS_SCORE_SUBSYSTEM_ELEVATOR_ELEVATOR_H_
