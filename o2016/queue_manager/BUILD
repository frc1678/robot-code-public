cc_library(
  name = 'queue_manager',
  srcs = ['queue_manager.cpp'],
  hdrs = ['queue_manager.h'],
  deps = [
    '//muan/queues:queues',
    '//muan/proto:stack_proto',
    '//muan/wpilib:wpilib_queues',
    '//o2016/subsystems/drivetrain:queue_types',
    '//o2016/subsystems/superstructure/turret:turret_queues',
    '//o2016/subsystems/superstructure/intake:intake_queues',
    '//o2016/subsystems/superstructure/catapult:catapult_queues',
    '//o2016/subsystems/superstructure/secondaries:secondaries_queues',
  ],
  visibility = ['//visibility:public']
)

cc_test (
  name = 'queue_manager_test',
  srcs = ['test_queue_manager.cpp'],
  deps = ['//third_party/google/googletest', ':queue_manager']
)
