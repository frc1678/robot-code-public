load("//muan/deploy:deploy.bzl", "muan_deploy")

cc_binary(
    name = "frc1678",
    srcs = ["main.cpp"],
    deps = [
        "//o2017/subsystems:subsystem_runner",
        "//o2017/citrus_robot:main",
        "//muan/wpilib:ds_sender",
        "//third_party/gflags:gflags",
        "//third_party/wpilibsuite:wpilib",
    ],
)

muan_deploy(
    name = "deploy",
    main = ":frc1678",
)
