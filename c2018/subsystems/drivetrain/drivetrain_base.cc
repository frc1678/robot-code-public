#include "c2018/subsystems/drivetrain/drivetrain_base.h"

#include "third_party/frc971/control_loops/drivetrain/drivetrain_config.h"

#include "c2018/subsystems/drivetrain/drivetrain_dog_motor_plant.h"
#include "c2018/subsystems/drivetrain/kalman_drivetrain_motor_plant.h"
#include "c2018/subsystems/drivetrain/polydrivetrain_dog_motor_plant.h"
#include "third_party/frc971/control_loops/state_feedback_loop.h"

using ::frc971::control_loops::drivetrain::DrivetrainConfig;

namespace c2018 {

namespace drivetrain {

const DrivetrainConfig &GetDrivetrainConfig() {
  static DrivetrainConfig kDrivetrainConfig{
      ::frc971::control_loops::drivetrain::ShifterType::SIMPLE_SHIFTER,
      ::frc971::control_loops::drivetrain::LoopType::CLOSED_LOOP,

      ::c2018::subsystems::drivetrain::MakeDrivetrainLoop,
      ::c2018::subsystems::drivetrain::MakeVelocityDrivetrainLoop,
      ::c2018::subsystems::drivetrain::MakeKFDrivetrainLoop,

      subsystems::drivetrain::kDt,
      subsystems::drivetrain::kRobotRadius,
      subsystems::drivetrain::kWheelRadius,
      subsystems::drivetrain::kV,

      subsystems::drivetrain::kHighGearRatio,
      subsystems::drivetrain::kLowGearRatio,
      true,
      0.0};

  return kDrivetrainConfig;
}

}  // namespace drivetrain

}  // namespace c2018
