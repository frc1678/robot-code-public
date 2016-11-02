cc_library(
  name = 'queue_manager',
  srcs = ['queue_manager.cpp'],
  hdrs = ['queue_manager.h'],
  deps = ['//muan/queues:queues',
          '//muan/proto:stack_proto',
          '//muan/wpilib:state_protos'],
)

cc_test (
  name = 'queue_manager_test',
  srcs = ['test_queue_manager.cpp'],
  deps = ['//third_party/google/googletest', ':queue_manager']
)
