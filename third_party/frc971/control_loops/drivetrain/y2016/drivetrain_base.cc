#include "third_party/frc971/control_loops/drivetrain/y2016/drivetrain_base.h"

#include "third_party/frc971/control_loops/drivetrain/drivetrain_config.h"

#include "third_party/frc971/control_loops/state_feedback_loop.h"
#include "third_party/frc971/control_loops/drivetrain/y2016/drivetrain_dog_motor_plant.h"
#include "third_party/frc971/control_loops/drivetrain/y2016/polydrivetrain_dog_motor_plant.h"
#include "third_party/frc971/control_loops/drivetrain/y2016/kalman_drivetrain_motor_plant.h"

using ::frc971::control_loops::drivetrain::DrivetrainConfig;

namespace y2016 {
namespace control_loops {
namespace drivetrain {

using ::frc971::constants::ShifterHallEffect;

const ShifterHallEffect kThreeStateDriveShifter{0.0, 0.0, 0.0, 0.0};

const DrivetrainConfig &GetDrivetrainConfig() {
  static DrivetrainConfig kDrivetrainConfig{
      ::frc971::control_loops::drivetrain::ShifterType::HALL_EFFECT_SHIFTER,
      ::frc971::control_loops::drivetrain::LoopType::CLOSED_LOOP,
      ::frc971::control_loops::drivetrain::GyroType::SPARTAN_GYRO,

      ::third_party::frc971::control_loops::drivetrain::y2016::MakeDrivetrainLoop,
      ::third_party::frc971::control_loops::drivetrain::y2016::MakeVelocityDrivetrainLoop,
      ::third_party::frc971::control_loops::drivetrain::y2016::MakeKFDrivetrainLoop,

      ::third_party::frc971::control_loops::drivetrain::y2016::kDt,
      ::third_party::frc971::control_loops::drivetrain::y2016::kRobotRadius,
      ::third_party::frc971::control_loops::drivetrain::y2016::kWheelRadius,
      ::third_party::frc971::control_loops::drivetrain::y2016::kV,

      ::third_party::frc971::control_loops::drivetrain::y2016::kHighGearRatio,
      ::third_party::frc971::control_loops::drivetrain::y2016::kLowGearRatio,
      kThreeStateDriveShifter,
      kThreeStateDriveShifter,
      true,
      0.0,
      0.4,
      1.0,
      kHighAlpha,
      kLowAlpha,
      kHighBeta,
      kLowBeta,
      kHighGamma,
      kLowGamma,
      kHighDelta,
      kLowDelta
  };

  return kDrivetrainConfig;
};

}  // namespace drivetrain
}  // namespace control_loops
}  // namespace y2016
