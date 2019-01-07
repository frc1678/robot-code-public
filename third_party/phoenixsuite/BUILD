licenses(["permissive"])

cc_library(
    name = "phoenix",
    srcs = [
        "api-cpp/linux/athena/static/libCTRE_Phoenix.a",
        "cci/linux/athena/static/libCTRE_PhoenixCCI.a",
        "wpiapi-cpp/linux/athena/static/libCTRE_Phoenix_WPI.a",
    ],
    hdrs = glob([
        "api-cpp/ctre/**/*.h",
        "cci/ctre/**/*.h",
        "core/ctre/**/*.h",
        "wpiapi-cpp/ctre/**/*.h",
    ]),
    includes = [
        "api-cpp/",
        "cci/",
        "core/",
        "wpiapi-cpp/",
    ],
    linkopts = ["-lpthread", "-pthread"],
    restricted_to = ["//tools:roborio"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/wpilibsuite:wpilib",
    ],
)
