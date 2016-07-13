package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'queues',
  hdrs = ['message_queue.h'],
  deps = ['//utils:math_utils'],
  linkstatic = 1
)

cc_test(
  name = 'queues_test',
  srcs = ['message_queue_test.cpp'],
  deps = ['@third_party//google/googletest', ':queues']
)
