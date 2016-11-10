cc_library(
  name = 'wpilib_interface',
  srcs = [
    'wpilib_interface.cpp',
  ],
  hdrs = [
    'wpilib_interface.h',
  ],
  deps = [
    '//o2016/queue_manager',
    '//o2016/subsystems/drivetrain:queue_types',
    '//o2016/subsystems/superstructure/turret:turret_queues',
    '//o2016/subsystems/superstructure/intake:intake_queues',
    '//o2016/subsystems/superstructure/catapult:catapult_queues',
    '//muan/wpilib:can_wrapper',
    '//third_party/wpilibsuite:wpilib',
  ],
  visibility = ['//visibility:public']
)
