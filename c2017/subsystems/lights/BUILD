load("/third_party/google/protobuf/protobuf", "cc_proto_library")

cc_library(
    name = "lights_queues",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":lights_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)

cc_proto_library(
    name = "lights_proto",
    srcs = ["lights.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_library(
    name = "lights",
    srcs = ["lights.cpp"],
    hdrs = ["lights.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":lights_queues",
        "//c2017/queue_manager",
        "//muan/units",
        "//muan/wpilib:wpilib_queues",
    ],
)

cc_test(
    name = "lights_test",
    srcs = ["lights_test.cpp"],
    deps = [
        ":lights",
        ":lights_queues",
        "//muan/utils:threading_utils",
        "//third_party/google/googletest",
    ],
)
