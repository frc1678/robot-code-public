load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
  name = "score_subsystem_proto",
  srcs = ["score_subsystem.proto"],
  default_runtime = "//third_party/google/protobuf",
  protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
  name = "score_subsystem_queues",
  hdrs = ["queue_types.h"],
  deps = [
  		"//muan/proto:stack_proto",
  		"//muan/queues",
      "score_subsystem_proto",
  	],
)

cc_library(
  name = "score_subsystem",
  srcs = ["score_subsystem.cpp"],
  hdrs = ["score_subsystem.h"],
  deps = [
    "//muan/queues:queue_manager",
    "//muan/wpilib:wpilib_queues",
    "//c2018/subsystems/score_subsystem/elevator:elevator_controller",
    ":score_subsystem_proto",
    ":score_subsystem_queues",
  ],

)

cc_test(
  name = "score_subsystem_test",
  srcs = ["score_subsystem_test.cpp"],
  deps = [
    ":score_subsystem",
    "//third_party/google/googletest",
  ],
)
