load("/third_party/google/protobuf/protobuf", "cc_proto_library")

cc_library(
    name = "climber_queues",
    hdrs = ["queue_types.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":climber_proto",
        "//muan/proto:stack_proto",
        "//muan/queues",
    ],
)

cc_library(
    name = "climber",
    srcs = ["climber.cpp"],
    hdrs = ["climber.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":climber_queues",
        "//c2017/queue_manager",
        "//muan/units",
        "//muan/utils:monitor",
        "//muan/wpilib:wpilib_queues",
    ],
)

cc_proto_library(
    name = "climber_proto",
    srcs = ["climber.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_test(
    name = "climber_test",
    srcs = ["climber_test.cpp"],
    deps = [
        ":climber",
        ":climber_queues",
        "//third_party/google/googletest",
    ],
)
