#ifndef O2018_SUBSYSTEMS_ARM_ARM_H_
#define O2018_SUBSYSTEMS_ARM_ARM_H_

#include "muan/logging/logger.h"
#include "muan/queues/queue_manager.h"
#include "muan/units/units.h"
#include "muan/utils/math_utils.h"
#include "muan/wpilib/queue_types.h"
#include "o2018/subsystems/arm/queue_types.h"

namespace o2018 {
namespace subsystems {
namespace arm {

constexpr double kMinAngle = 0.0;
constexpr double kMaxAngle = 90 * (M_PI / 180);

constexpr double kIntakeVoltage = 12;
constexpr double kHoldingVoltage = 1.5;
constexpr double kSlowOuttakeVoltage = -6;
constexpr double kFastOuttakeVoltage = -9;

constexpr double kCalibVoltage = 0;

constexpr double kEncoderFaultMinVoltage = 6;
constexpr double kEncoderFaultTicksAllowed = 100;

constexpr int kNumHasCubeTicks = 150;

constexpr double kArmFF = 3.48;     // V
constexpr double kArmCubeFF = 3.9;  // V

class Arm {
 public:
  Arm();
  void SetGoal(double angle, IntakeMode intake_goal);
  void Update();

  inline bool is_calibrated() const { return calibrated_; }

 private:
  double CalculateFeedForwards(bool has_cube, double theta);
  void UpdateProfiledGoal(bool outputs_enabled);
  void ReadInputs();

  ArmGoalQueue::QueueReader goal_reader_;
  ArmInputQueue::QueueReader input_reader_;
  ArmStatusQueue* status_queue_;
  ArmOutputQueue* output_queue_;

  muan::wpilib::DriverStationQueue::QueueReader ds_status_reader_;

  double prev_position_ = 0;
  double unprofiled_goal_ = 0;
  double profiled_goal_ = 0;

  int has_cube_for_ticks_ = kNumHasCubeTicks;
  int num_encoder_fault_ticks_ = 0;
  bool encoder_fault_detected_ = false;
  bool calibrated_ = false;

  IntakeMode intake_goal_ = INTAKE_NONE;
};

}  // namespace arm
}  // namespace subsystems
}  // namespace o2018

#endif  // O2018_SUBSYSTEMS_ARM_ARM_H_
