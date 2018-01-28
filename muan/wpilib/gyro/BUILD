load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "gyro_queues",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [":gyro_protos"],
)

cc_library(
    name = "gyro",
    srcs = [
        "gyro_interface.cpp",
        "gyro_reader.cpp",
    ],
    hdrs = [
        "gyro_interface.h",
        "gyro_reader.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":gyro_queues",
        "//muan/logging:logger",
        "//muan/proto:stack_proto",
        "//muan/queues",
        "//muan/units",
        "//muan/utils:history",
        "//muan/utils:threading_utils",
        "//muan/wpilib:wpilib_queues",
        "//third_party/aos/common/util:phased_loop",
        "//third_party/aos/linux_code:init",
        "//third_party/wpilibsuite:wpilib",
    ],
    restricted_to = ["//tools:roborio"],
)

cc_proto_library(
    name = "gyro_protos",
    srcs = ["gyro.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)
