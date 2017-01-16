load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')
package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'shooter_queues',
  hdrs = ['queue_types.h'],
  deps = [
    'shooter_proto',
    '//muan/queues',
    '//muan/proto:stack_proto'
  ]
)

cc_proto_library(
  name = 'shooter_proto',
  srcs = ['shooter.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)
