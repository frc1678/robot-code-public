cc_library(
  name = 'subsystem_runner',
  srcs = ['subsystem_runner.cpp'],
  hdrs = ['subsystem_runner.h'],
  deps = [
    '//c2017/wpilib:wpilib_interface',
    '//c2017/subsystems/drivetrain:drivetrain_base',
    '//third_party/frc971/control_loops/drivetrain:drivetrain_lib',
    '//third_party/aos/linux_code:init',
    '//third_party/aos/common/util:phased_loop',
    '//third_party/aos/common:time'
  ],
  visibility = ['//visibility:public']
)

