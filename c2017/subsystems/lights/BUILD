load('/third_party/google/protobuf/protobuf', 'cc_proto_library')

cc_library(
    name = "lights_queues",
    hdrs = ["queue_types.h"],
    deps = [":lights_proto", 
          "//muan/proto:stack_proto",
          "//muan/queues:queues"],
    visibility = ["//visibility:public"]
)

cc_proto_library(
    name = "lights_proto",
    srcs = ["lights.proto"],
    protoc = "//third_party/google/protobuf:protoc",
    default_runtime = "//third_party/google/protobuf"
)

cc_library(
    name = "lights",
    srcs = ["lights.cpp"],
    hdrs = ["lights.h"],
    visibility = ["//visibility:public"],
    deps = [":lights_queues",
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
        "//third_party/google/googletest",
    ], 
)
