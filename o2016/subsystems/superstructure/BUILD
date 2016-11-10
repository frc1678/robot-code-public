load('//third_party/google/protobuf:protobuf.bzl', 'cc_proto_library')
package(default_visibility = ['//visibility:public'])

cc_library(
  name = "superstructure",
  hdrs = ["superstructure.h"],
  srcs = ["superstructure.cpp"],
  deps = [":queue_types",
          "//muan/proto:stack_proto",
          "//muan/queues",
          "//o2016/subsystems/superstructure/turret:turret_queues",
          "//o2016/subsystems/superstructure/intake:intake_queues",
          "//o2016/subsystems/superstructure/catapult:catapult_queues",
          "//o2016/subsystems/superstructure/turret:pid_turret_controller",
          "//o2016/subsystems/superstructure/intake:intake",
          "//o2016/subsystems/superstructure/catapult:catapult",
          "//o2016/queue_manager:queue_manager"]
)

cc_library(
  name = "queue_types",
  hdrs = ["queue_types.h"],
  deps = [":superstructure_proto"]
)

cc_proto_library(
  name = 'superstructure_proto',
  srcs = ['superstructure.proto'],
  protoc = '//third_party/google/protobuf:protoc',
  default_runtime = '//third_party/google/protobuf'
)
