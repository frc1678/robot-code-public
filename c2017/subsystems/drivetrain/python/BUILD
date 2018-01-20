package(default_visibility = ["//visibility:public"])

licenses(["notice"])

py_binary(
    name = "drivetrain",
    srcs = [
        "drivetrain.py",
    ],
    main = "drivetrain.py",
    deps = [
        "//third_party/frc971/control_loops/python:drivetrain",
    ],
)

py_binary(
    name = "polydrivetrain",
    srcs = [
        "drivetrain.py",
        "polydrivetrain.py",
    ],
    main = "polydrivetrain.py",
    deps = [
        # '//external:python-gflags',
        # '//external:python-glog',
        "//third_party/frc971/control_loops/python:polydrivetrain",
    ],
)

py_library(
    name = "polydrivetrain_lib",
    srcs = [
        "drivetrain.py",
        "polydrivetrain.py",
    ],
    deps = [
        # '//external:python-gflags',
        # '//external:python-glog',
        "//third_party/frc971/control_loops/python:controls",
    ],
)
