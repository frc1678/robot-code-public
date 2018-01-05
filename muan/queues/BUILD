load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")
package(default_visibility = ["//visibility:public"])

cc_library(
    name = "queues",
    srcs = ["message_queue.hpp"],
    hdrs = ["message_queue.h"],
    linkstatic = 1,
    deps = [
        "//muan/utils:math_utils",
        "//muan/utils:hash",
        "//muan/proto:stack_proto",
        "//third_party/aos/common:mutex",
        "//third_party/optional",
    ],
)

cc_library(
    name = "queue_manager",
    srcs = ["queue_manager.cpp"],
    hdrs = ["queue_manager.h"],
    deps = [
        ":queues",
        "//muan/utils:string_utils",
        "//muan/logging:logger",
        "//muan/webdash:server",
    ],
)

cc_proto_library(
    name = "queue_test_proto",
    srcs = ["test_proto.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_test(
    name = "queues_test",
    srcs = ["message_queue_test.cpp"],
    deps = [
        ":queues",
        ":queue_manager",
        ":queue_test_proto",
        "//muan/proto:stack_proto",
        "//third_party/google/googletest",
    ],
)
