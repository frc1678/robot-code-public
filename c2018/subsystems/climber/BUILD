load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
  name = "climber_proto",
  srcs = ["climber.proto"],
  default_runtime = "//third_party/google/protobuf",
  protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
  name = "climber_subsystem",
  srcs = ["climber.cpp"],
  hdrs = ["climber.h"],
  deps = [
      ":climber_proto",
      ":climber_queues",
      "//muan/wpilib:wpilib_queues",
      "//muan/queues:queue_manager",
      "//c2018/subsystems/climber/batter",
      "//muan/wpilib:state_protos",
  ],
)

cc_library(
  name = "climber_queues",
  hdrs = ["queue_types.h"],
  deps = [
      ":climber_proto",
      "//muan/proto:stack_proto",
      "//muan/queues",
  ],
)

cc_test(
  name = "climber_test",
  srcs = ["climber_test.cpp"],
  deps = [
      ":climber_subsystem",
      "//third_party/google/googletest",
  ],
)
