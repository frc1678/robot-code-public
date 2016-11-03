load('/third_party/google/protobuf/protobuf', 'cc_proto_library')

cc_library(
  name = 'queue_types',
  hdrs = ['queue_types.h'],
  deps = ['drivetrain_protos', '//muan/queues', '//muan/proto:stack_proto'],
  visibility = ['//visibility:public']
)

cc_proto_library(
  name = 'drivetrain_protos',
  srcs = ['drivetrain.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)
