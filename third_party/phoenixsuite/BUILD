licenses(["permissive"])

cc_library(
    name = "driver",
    srcs = glob([
        "libraries/driver/lib/**/*.a"
    ]),
    hdrs = glob([
        "libraries/driver/include/**/*.h",
    ]),
    includes = [
        "libraries/driver/include",
    ],
    restricted_to = ["//tools:roborio"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/wpilibsuite:wpilib",
    ],
)

cc_library(
    name = "phoenix",
    srcs = glob([
        "cpp/src/**/*.cpp",
    ]),
    hdrs = glob([
        "cpp/include/**/*.h",
    ]),
    includes = [
        "cpp/include/",
    ],
    restricted_to = ["//tools:roborio"],
    visibility = ["//visibility:public"],
    deps = [
        "//third_party/wpilibsuite:wpilib",
        ":driver",
    ],
)
