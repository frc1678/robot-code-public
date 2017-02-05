#include "c2017/subsystems/drivetrain/drivetrain_base.h"

#include "third_party/frc971/control_loops/drivetrain/drivetrain_config.h"

#include "c2017/subsystems/drivetrain/drivetrain_dog_motor_plant.h"
#include "c2017/subsystems/drivetrain/kalman_drivetrain_motor_plant.h"
#include "c2017/subsystems/drivetrain/polydrivetrain_dog_motor_plant.h"
#include "third_party/frc971/control_loops/state_feedback_loop.h"

using ::frc971::control_loops::drivetrain::DrivetrainConfig;

namespace c2017 {

namespace drivetrain {

const DrivetrainConfig &GetDrivetrainConfig() {
  static DrivetrainConfig kDrivetrainConfig{
      ::frc971::control_loops::drivetrain::ShifterType::SIMPLE_SHIFTER,
      ::frc971::control_loops::drivetrain::LoopType::CLOSED_LOOP,

      ::c2017::subsystems::drivetrain::MakeDrivetrainLoop,
      ::c2017::subsystems::drivetrain::MakeVelocityDrivetrainLoop,
      ::c2017::subsystems::drivetrain::MakeKFDrivetrainLoop,

      subsystems::drivetrain::kDt,
      subsystems::drivetrain::kRobotRadius,
      subsystems::drivetrain::kWheelRadius,
      subsystems::drivetrain::kV,

      subsystems::drivetrain::kGearRatio,
      subsystems::drivetrain::kGearRatio,
      true,
      0.0};

  return kDrivetrainConfig;
}

}  // namespace drivetrain

}  // namespace c2017
