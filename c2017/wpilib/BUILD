load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "wpilib_queues",
    hdrs = [
        "queue_types.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":wpilib_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)

cc_proto_library(
    name = "wpilib_proto",
    srcs = ["wpilib_superstructure.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
    visibility = ["//visibility:public"],
)

cc_library(
    name = "superstructure_interface",
    srcs = [
        "superstructure_interface.cpp",
    ],
    hdrs = [
        "superstructure_interface.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//c2017/queue_manager",
        "//muan/utils:math_utils",
        "//muan/wpilib:can_wrapper",
        "//muan/wpilib:pcm_wrapper",
        "//third_party/wpilibsuite:wpilib",
    ],
)

cc_library(
    name = "wpilib_interface",
    srcs = [
        "wpilib_interface.cpp",
    ],
    hdrs = [
        "wpilib_interface.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        ":drivetrain_interface",
        ":superstructure_interface",
        ":lights_interface",
        "//c2017/queue_manager",
        "//muan/wpilib:can_wrapper",
        "//muan/wpilib/gyro",
        "//third_party/wpilibsuite:wpilib",
    ],
)

cc_library(
    name = "drivetrain_interface",
    srcs = [
        "drivetrain_interface.cpp",
    ],
    hdrs = [
        "drivetrain_interface.h",
    ],
    deps = [
        "//c2017/queue_manager",
        "//muan/utils:math_utils",
        "//muan/wpilib/gyro",
        "//third_party/wpilibsuite:wpilib",
    ],
)

cc_library(
    name = "lights_interface",
    srcs = [
        "lights_interface.cpp",
    ],
    hdrs = [
        "lights_interface.h",
    ],
    deps = [
        "//c2017/subsystems/lights:lights",
        "//third_party/wpilibsuite:wpilib",
        "//c2017/queue_manager",
    ],
)
