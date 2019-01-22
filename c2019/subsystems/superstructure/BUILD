load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

package(default_visibility = ["//visibility:public"])

cc_proto_library(
    name = "superstructure_proto",
    srcs = ["superstructure.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
    name = "superstructure",
    srcs = ["superstructure.cpp"],
    hdrs = ["superstructure.h"],
    deps = [":superstructure_queues",
    "//c2019/subsystems/superstructure/elevator:elevator",
    "//c2019/subsystems/superstructure/wrist:wrist",
    "//c2019/subsystems/superstructure/ground_hatch_intake:ground_hatch_intake",
    "//c2019/subsystems/superstructure/hatch_intake:hatch_intake",
    "//c2019/subsystems/superstructure/cargo_intake:cargo_intake",
    "//c2019/subsystems/superstructure/winch:winch",
    "//muan/queues:queue_manager",
    "//muan/wpilib:wpilib_queues",
    ]
)

cc_library(
    name = "superstructure_queues",
    hdrs = ["queue_types.h"],
    deps = [
        "superstructure_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ]
)

cc_test(
    name = "superstructure_test",
    srcs = ["superstructure_test.cpp"],
    deps = [
        ":superstructure",
        "//third_party/google/googletest",
    ],
)
