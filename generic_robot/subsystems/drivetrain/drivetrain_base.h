#ifndef GENERIC_ROBOT_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_BASE_H_
#define GENERIC_ROBOT_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_BASE_H_

#include "third_party/frc971/control_loops/drivetrain/drivetrain_config.h"

namespace generic_robot {

namespace drivetrain {

const ::frc971::control_loops::drivetrain::DrivetrainConfig&
GetDrivetrainConfig();

}  // namespace drivetrain

}  // namespace generic_robot

#endif  // GENERIC_ROBOT_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_BASE_H_
