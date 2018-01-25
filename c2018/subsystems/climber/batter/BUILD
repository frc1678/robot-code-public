cc_library(
    name = "batter",
    srcs = ["batter.cpp"],
    hdrs = ["batter.h"],
    visibility = ["//visibility:public"],
    deps = [],
)

cc_test(
    name = "batter_test",
    srcs = ["batter_test.cpp"],
    deps = [
        ":batter",
        "//third_party/google/googletest",
    ],
)
