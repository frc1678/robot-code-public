#include "generic_robot/subsystems/drivetrain/drivetrain_base.h"

#include "third_party/frc971/control_loops/drivetrain/drivetrain_config.h"

#include "generic_robot/subsystems/drivetrain/drivetrain_dog_motor_plant.h"
#include "generic_robot/subsystems/drivetrain/kalman_drivetrain_motor_plant.h"
#include "generic_robot/subsystems/drivetrain/polydrivetrain_dog_motor_plant.h"
#include "third_party/frc971/control_loops/state_feedback_loop.h"

using ::frc971::control_loops::drivetrain::DrivetrainConfig;

namespace generic_robot {

namespace drivetrain {

const DrivetrainConfig &GetDrivetrainConfig() {
  static DrivetrainConfig kDrivetrainConfig{
      ::frc971::control_loops::drivetrain::ShifterType::SIMPLE_SHIFTER,
      ::frc971::control_loops::drivetrain::LoopType::CLOSED_LOOP,

      ::generic_robot::subsystems::drivetrain::MakeDrivetrainLoop,
      ::generic_robot::subsystems::drivetrain::MakeVelocityDrivetrainLoop,
      ::generic_robot::subsystems::drivetrain::MakeKFDrivetrainLoop,

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

}  // namespace generic_robot
