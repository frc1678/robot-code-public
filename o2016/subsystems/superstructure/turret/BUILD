load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')
package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'turret_queues',
  hdrs = ['queue_types.h'],
  deps = ['turret_proto', '//muan/queues', '//muan/proto:stack_proto']
)

cc_library(
  name = 'pid_turret_controller',
  hdrs = ['pid_turret_controller.h'],
  srcs = ['pid_turret_controller.cpp'],
  deps = [':turret_proto',
          ':turret_queues',
          '//muan/control:pid_controller',
          '//muan/utils/offset_profiled_controller:pot_calibration']
)

cc_proto_library(
  name = 'turret_proto',
  srcs = ['turret.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)
