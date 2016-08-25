package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'queues',
  srcs = ['message_queue.hpp'],
  hdrs = ['message_queue.h'],
  deps = ['//muan/utils:math_utils', '@third_party//optional'],
  linkstatic = 1
)

cc_test(
  name = 'queues_test',
  srcs = ['message_queue_test.cpp'],
  deps = ['@third_party//google/googletest', ':queues']
)
