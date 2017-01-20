cc_library(
  name = 'queue_manager',
  srcs = ['queue_manager.cpp'],
  hdrs = ['queue_manager.h'],
  deps = [
    '//muan/queues:queues',
    '//muan/proto:stack_proto',
    '//muan/wpilib:wpilib_queues',
    '//muan/wpilib/gyro:gyro_queues',
    '//third_party/frc971/control_loops/drivetrain:drivetrain_lib',
    '//c2017/subsystems/superstructure/shooter:shooter_queues'
  ],
  visibility = ['//visibility:public']
)

cc_test (
  name = 'queue_manager_test',
  srcs = ['test_queue_manager.cpp'],
  deps = ['//third_party/google/googletest', ':queue_manager']
)
