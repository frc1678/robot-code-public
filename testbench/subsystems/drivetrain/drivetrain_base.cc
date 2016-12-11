#include "testbench/subsystems/drivetrain/drivetrain_base.h"

#include "third_party/frc971/control_loops/drivetrain/drivetrain_config.h"

#include "testbench/subsystems/drivetrain/drivetrain_dog_motor_plant.h"
#include "testbench/subsystems/drivetrain/kalman_drivetrain_motor_plant.h"
#include "testbench/subsystems/drivetrain/polydrivetrain_dog_motor_plant.h"
#include "third_party/frc971/control_loops/state_feedback_loop.h"

using ::frc971::control_loops::drivetrain::DrivetrainConfig;

namespace testbench {

namespace drivetrain {

const DrivetrainConfig &GetDrivetrainConfig() {
  static DrivetrainConfig kDrivetrainConfig{
      ::frc971::control_loops::drivetrain::ShifterType::HALL_EFFECT_SHIFTER,
      ::frc971::control_loops::drivetrain::LoopType::CLOSED_LOOP,

      ::testbench::subsystems::drivetrain::MakeDrivetrainLoop,
      ::testbench::subsystems::drivetrain::MakeVelocityDrivetrainLoop,
      ::testbench::subsystems::drivetrain::MakeKFDrivetrainLoop,

      subsystems::drivetrain::kDt,
      subsystems::drivetrain::kRobotRadius,
      subsystems::drivetrain::kWheelRadius,
      subsystems::drivetrain::kV,

      subsystems::drivetrain::kHighGearRatio,
      subsystems::drivetrain::kLowGearRatio,
      true,
      0.0};

  return kDrivetrainConfig;
};

}  // namespace drivetrain

}  // namespace testbench
