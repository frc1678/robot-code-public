load('/third_party/google/protobuf/protobuf', 'cc_proto_library')

cc_library(
  name = 'climber_queues',
  hdrs = ['queue_types.h'],
  deps = [':climber_proto', '//muan/proto:stack_proto', '//muan/queues:queues'],
  visibility = ['//visibility:public']
)

cc_library(
  name = 'climber',
  hdrs = ['climber.h'],
  srcs = ['climber.cpp'],
  deps = ['//muan/units:units', ':climber_queues' ],
  visibility = ['//visibility:public']
)
cc_proto_library(
  name = 'climber_proto',
  srcs = ['climber.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)

cc_test(
  name = 'climber_test',
  srcs = ['climber_test.cpp'],
  deps = [ ':climber', '//third_party/google/googletest:googletest']
)
