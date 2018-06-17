package(default_visibility = ["//visibility:public"])

cc_library(
    name = "talon_wrapper",
    srcs = [
        "talon_wrapper.cpp",
    ],
    hdrs = ["talon_wrapper.h"],
    deps = [
        "//muan/units",
    ],
    includes = ["."],
)

cc_library(
    name = "victor_wrapper",
    srcs = [
        "victor_wrapper.cpp",
    ],
    hdrs = ["victor_wrapper.h"],
    deps = [
        "//muan/units",
    ],
    includes = ["."],
)

cc_test(
    name = "talon_wrapper_test",
    srcs = ["talon_wrapper_test.cpp"],
    deps = [
        ":talon_wrapper",
        "//third_party/google/googletest",
    ],
)

cc_test(
    name = "victor_wrapper_test",
    srcs = ["victor_wrapper_test.cpp"],
    deps = [
        ":victor_wrapper",
        "//third_party/google/googletest",
    ],
)
