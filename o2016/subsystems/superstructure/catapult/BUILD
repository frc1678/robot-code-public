cc_library(
  name = 'catapult',
  srcs = ['catapult.cpp'],
  hdrs = [
    'catapult.h',
    'catapult_messages.h'
  ],
  deps = [
    '//o2016/subsystems/superstructure/catapult/scoop:scoop',
    '//o2016/subsystems/superstructure/catapult/stop:stop',
    '//muan/units:units',
    '//muan/queues:queues'
  ]
)

cc_test(
  name = 'catapult_test',
  srcs = ['catapult_test.cpp'],
  deps = [
    ':catapult',
    '//third_party/google/googletest:googletest'
  ]
)
