cc_library(
    name = "wpilib_interface",
    srcs = [
        "wpilib_interface.cpp",
        "drivetrain_interface.cpp",
        "superstructure_interface.cpp",
    ],
    hdrs = [
        "wpilib_interface.h",
        "drivetrain_interface.h",
        "superstructure_interface.h",
    ],
    visibility = ["//visibility:public"],
    deps = [
        "//generic_robot/queue_manager",
        "//muan/wpilib:can_wrapper",
        "//muan/wpilib/gyro:gyro",
        "//third_party/wpilibsuite:wpilib",
    ],
)
