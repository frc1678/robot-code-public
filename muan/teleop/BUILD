load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "joystick",
    srcs = [
        "button.cpp",
        "joystick.cpp",
    ],
    hdrs = [
        "button.h",
        "joystick.h",
    ],
    visibility = ["//visibility:public"],
    deps = ["//third_party/wpilibsuite:wpilib"],
)

cc_library(
    name = "joystick_queues",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":joystick_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)

cc_proto_library(
    name = "joystick_proto",
    srcs = ["joystick.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)
