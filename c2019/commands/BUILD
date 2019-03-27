load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_proto_library(
    name = "commands_proto",
    srcs = ["commands.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
    name = "command_queues",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":commands_proto",
        "//muan/proto:stack_proto",
        "//muan/queues:queue_manager",
	"//c2019/subsystems/limelight:limelight_queues",
    ],
)

cc_library(
    name = "commands",
    deps = [
        ":command_base",
        ":command_queues",
        ":rocket",
    ],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "command_base",
    srcs = ["command_base.cpp"],
    hdrs = ["command_base.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":command_queues",
        "//muan/logging:logger",
        "//muan/queues:queue_manager",
        "//muan/subsystems/drivetrain:drivetrain_queues",
        "//muan/wpilib:wpilib_queues",
        "//c2019/subsystems/limelight:limelight_queues",
        "//c2019/subsystems/superstructure:superstructure_queues",
        "//c2019/subsystems/superstructure/cargo_intake:cargo_intake_queues",
    ],
)

cc_test(
    name = "command_base_test",
    srcs = ["command_base_test.cpp"],
    linkopts = ["-lpthread"],
    deps = [
        ":command_base",
        "//third_party/google/googletest",
    ],
)

cc_library(
    name = "rocket",
    srcs = ["rocket.cpp"],
    hdrs = ["rocket.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":command_base",
    ],
)
