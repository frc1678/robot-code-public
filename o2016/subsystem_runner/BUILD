cc_library(
  name = 'subsystem_runner',
  srcs = ['subsystem_runner.cpp'],
  hdrs = ['subsystem_runner.h'],
  deps = [
    '//o2016/wpilib:wpilib_interface',
    '//o2016/subsystems/superstructure/catapult:catapult',
    '//o2016/subsystems/superstructure/turret:pid_turret_controller',
    '//o2016/subsystems/superstructure/intake',
    '//o2016/subsystems/superstructure:superstructure',
    '//third_party/aos/linux_code:init',
    '//third_party/aos/common/util:phased_loop',
    '//third_party/aos/common:time',
    '//o2016/subsystems/drivetrain',
  ],
  visibility = ['//visibility:public']
)
