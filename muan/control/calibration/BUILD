cc_library(
    name = "hall_calibration",
    srcs = ["hall_calibration.cpp"],
    hdrs = ["hall_calibration.h"],
    visibility = ["//visibility:public"],
)

cc_library(
    name = "pot_calibration",
    srcs = ["pot_calibration.cpp"],
    hdrs = ["pot_calibration.h"],
    visibility = ["//visibility:public"],
)

cc_test(
    name = "hall_calibration_test",
    srcs = [
        "hall_calibration_test.cpp",
    ],
    linkopts = ["-lm"],
    deps = [
        ":hall_calibration",
        "//muan/utils:math_utils",
        "//third_party/google/googletest",
    ],
)

cc_test(
    name = "pot_calibration_test",
    srcs = [
        "pot_calibration_test.cpp",
    ],
    linkopts = ["-lm"],
    deps = [
        ":pot_calibration",
        "//muan/utils:math_utils",
        "//third_party/google/googletest",
    ],
)
