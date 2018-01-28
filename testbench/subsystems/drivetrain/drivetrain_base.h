#ifndef TESTBENCH_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_BASE_H_
#define TESTBENCH_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_BASE_H_

#include "third_party/frc971/control_loops/drivetrain/drivetrain_config.h"

namespace testbench {

namespace drivetrain {

const ::frc971::control_loops::drivetrain::DrivetrainConfig&
GetDrivetrainConfig();

}  // namespace drivetrain

}  // namespace testbench

#endif  // TESTBENCH_SUBSYSTEMS_DRIVETRAIN_DRIVETRAIN_BASE_H_
