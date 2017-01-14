load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')
package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'trigger_queues',
  hdrs = ['queue_types.h'],
  deps = ['trigger_proto', '//muan/queues', '//muan/proto:stack_proto']
)

cc_proto_library(
  name = 'trigger_proto',
  srcs = ['trigger.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)
