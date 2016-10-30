load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')

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
    '//muan/queues:queues',
    ':catapult_proto'
  ]
)

cc_proto_library(
  name = 'catapult_proto',
  srcs = ['catapult.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)

cc_test(
  name = 'catapult_test',
  srcs = ['catapult_test.cpp'],
  deps = [
    ':catapult',
    '//third_party/google/googletest:googletest'
  ]
)
