load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "magazine",
    srcs = ["magazine.cpp"],
    hdrs = ["magazine.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":magazine_queues",
        "//c2017/queue_manager",
        "//muan/control:ramping",
        "//muan/wpilib:wpilib_queues",
    ],
)

cc_proto_library(
    name = "magazine_proto",
    srcs = ["magazine.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
    name = "magazine_queues",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":magazine_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)

cc_test(
    name = "magazine_test",
    srcs = ["magazine_test.cpp"],
    deps = [
        ":magazine",
        "//third_party/google/googletest",
    ],
)
