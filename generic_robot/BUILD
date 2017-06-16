load("//muan/deploy:deploy.bzl", "muan_deploy")

cc_binary(
    name = "frc1678",
    srcs = ["main.cpp"],
    deps = [
        "//generic_robot/subsystems:subsystem_runner",
        "//generic_robot/citrus_robot:main",
        "//muan/wpilib:ds_sender",
        "//third_party/gflags:gflags",
        "//third_party/wpilibsuite:wpilib",
    ],
)

muan_deploy(
    name = "deploy",
    main = ":frc1678",
)
