cc_library(
    name = "citrus_robot",
    srcs = ["citrus_robot.cpp"],
    hdrs = ["citrus_robot.h"],
    visibility = ["//visibility:public"],
    deps = [
        # Joystick needs to be first because of janky WPILib build. I don't know
        # why it does this. Possibly a bazel bug.
        # TODO(Kyle) Figure this out and fix it.
        "//muan/teleop:joystick",
        "//muan/wpilib:ds_sender",
        "//muan/utils:threading_utils",
        "//c2017/lemonscript",
        "//c2017/queue_manager",
        "//c2017/subsystems/drivetrain:drivetrain_base",
        "//third_party/aos/common/util:phased_loop",
        "//third_party/lemonscript",
    ],
    restricted_to = ["//tools:roborio"],
)
