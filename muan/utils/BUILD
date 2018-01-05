package(default_visibility = ["//visibility:public"])

load("//third_party/google/protobuf:protobuf.bzl", "cc_proto_library")

cc_library(
    name = "math_utils",
    srcs = ["math_utils.cpp"],
    hdrs = ["math_utils.h"],
    linkopts = ["-lm"],
    linkstatic = 1,
    deps = [
        "//muan/units",
        "//third_party/eigen",
    ],
)

cc_library(
    name = "history",
    hdrs = ["history.h"],
    linkstatic = 1,
    deps = [
        "//muan/units",
        "//third_party/aos/common:die",
    ],
)

cc_library(
    name = "disk_brake",
    srcs = ["disk_brake.cpp"],
    hdrs = ["disk_brake.h"],
    deps = [
        ":math_utils",
        "//muan/units",
        "//third_party/aos/common:time",
    ],
)

cc_library(
    name = "linear_interpolation",
    srcs = ["linear_interpolation.hpp"],
    hdrs = ["linear_interpolation.h"],
    deps = ["//third_party/aos/common:die"],
)

cc_library(
    name = "proto_utils",
    srcs = ["proto_utils.cpp"],
    hdrs = ["proto_utils.h"],
    deps = [
        "//third_party/aos/common:die",
        "//third_party/google/protobuf",
    ],
)

cc_library(
    name = "monitor",
    srcs = ["monitor.cpp"],
    hdrs = ["monitor.h"],
    deps = [":history"]
)

cc_library(
    name = "threading_utils",
    srcs = [
        "threading_utils.hpp",
        "threading_utils.cpp"
    ],
    hdrs = ["threading_utils.h"],
    deps = [
        "//third_party/aos/common:time",
        "//third_party/aos/linux_code:init",
    ],
)

cc_proto_library(
    name = "test_proto",
    srcs = ["test_proto.proto"],
    default_runtime = "//third_party/google/protobuf",
    protoc = "//third_party/google/protobuf:protoc",
)

cc_test(
    name = "proto_utils_test",
    srcs = ["proto_utils_test.cpp"],
    deps = [
        ":proto_utils",
        ":test_proto",
        "//third_party/google/googletest",
    ],
)

cc_test(
    name = "linear_interpolation_test",
    srcs = ["linear_interpolation_test.cpp"],
    deps = [
        ":disk_brake",
        ":linear_interpolation",
        "//third_party/google/googletest",
    ],
)

cc_test(
    name = "history_test",
    srcs = ["history_test.cpp"],
    deps = [
        ":history",
        "//third_party/google/googletest",
    ],
)

cc_test(
    name = "monitor_test",
    srcs = ["monitor_test.cpp"],
    deps = [
        ":monitor",
        "//third_party/google/googletest:googletest"
    ],
    linkopts = ["-lm"]
)

cc_test(
    name = "threading_utils_test",
    srcs = ["threading_utils_test.cpp"],
    deps = [
        ":threading_utils",
        "//third_party/google/googletest:googletest"
    ],
)

cc_library(
    name = "hash",
    srcs = ["hash.cpp"],
    hdrs = ["hash.h"],
)

cc_library(
    name = "string_utils",
    srcs = ["string_utils.cpp"],
    hdrs = ["string_utils.h"],
    linkstatic = 1,
)

cc_test(
    name = "string_utils_test",
    srcs = ["string_utils_test.cpp"],
    deps = [
        ":string_utils",
        "//third_party/google/googletest:googletest",
    ],
)
