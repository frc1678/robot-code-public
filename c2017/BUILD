load("//muan/deploy:deploy.bzl", "muan_deploy")

cc_binary(
    name = "frc1678",
    srcs = ["main.cpp"],
    deps = [
        "//c2017/subsystems:subsystem_runner",
        "//c2017/vision/robot:reader",
        "//c2017/citrus_robot:citrus_robot",
        "//muan/teleop:joystick",
        "//third_party/wpilibsuite:wpilib",
        "//muan/webdash:server",
        "//third_party/gflags:gflags"
    ],
    data = [
        "//c2017/lemonscript/auto:auto_modes",
        "//muan/webdash:webdash_html"
    ],
    restricted_to = ["//tools:roborio"],
)

muan_deploy(
    name = "deploy",
    main = ":frc1678",
)
