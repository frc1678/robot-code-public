#ifndef MUAN_SUBSYSTEMS_DRIVETRAIN_OPEN_LOOP_DRIVE_H_
#define MUAN_SUBSYSTEMS_DRIVETRAIN_OPEN_LOOP_DRIVE_H_

#include "muan/subsystems/drivetrain/drivetrain_config.h"
#include "muan/subsystems/drivetrain/queue_types.h"

namespace muan {
namespace subsystems {
namespace drivetrain {

constexpr double kThrottleDeadband = 0.02;
constexpr double kWheelDeadband = 0.02;

constexpr double kHighWheelNonLinearity = 0.4;
constexpr double kLowWheelNonLinearity = 0.5;

constexpr double kHighNegInertiaScalar = 4.;

constexpr double kLowNegInertiaThreshold = 0.65;
constexpr double kLowNegInertiaTurnScalar = 3.5;
constexpr double kLowNegInertiaCloseScalar = 4.0;
constexpr double kLowNegInertiaFarScalar = 5.0;

constexpr double kHighSensitivity = 0.95;
constexpr double kLowSensitivity = 0.65;

constexpr double kQuickStopDeadband = 0.5;
constexpr double kQuickStopWeight = 0.1;
constexpr double kQuickStopScalar = 5.0;

class OpenLoopDrive {
 public:
  explicit OpenLoopDrive(DrivetrainConfig dt_config) : dt_config_(dt_config) {}
  void Update(OutputProto* output);
  void SetGoal(const GoalProto& goal);

 private:
  double throttle_;
  double wheel_;
  bool quickturn_;
  bool high_gear_;

  double old_wheel_ = 0.;
  double quick_stop_accum_ = 0.;
  double neg_inertia_accum_ = 0.;

  DrivetrainConfig dt_config_;
};

}  // namespace drivetrain
}  // namespace subsystems
}  // namespace muan

#endif  // MUAN_SUBSYSTEMS_DRIVETRAIN_OPEN_LOOP_DRIVE_H_
