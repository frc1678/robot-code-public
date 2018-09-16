package(default_visibility = ["//visibility:public"])

py_library(
    name = "python_controls",
    srcs = [
        "controls.py",
        "state_space_controller.py",
        "state_space_gains.py",
        "state_space_observer.py",
        "state_space_plant.py",
        "state_space_scenario.py",
        "state_space_writer.py",
        "trapezoidal_profile.py",
    ],
)

py_binary(
    name = "python_controls_demo",
    srcs = ["state_space_demo.py"],
    main = "state_space_demo.py",
    deps = [":python_controls"],
)

cc_library(
    name = "pose",
    srcs = [
        "pose.cpp",
    ],
    hdrs = ["pose.h"],
    linkopts = ["-lm"],
    deps = [
        "//third_party/eigen",
    ],
)

cc_library(
    name = "spline",
    srcs = ["spline.cpp"],
    hdrs = ["spline.h"],
    linkopts = ["-lm"],
    deps = [
        ":pose",
        "//muan/utils:math_utils",
        "//third_party/eigen",
    ],
)

cc_library(
    name = "drivetrain_model",
    srcs = ["drivetrain_model.cpp"],
    hdrs = ["drivetrain_model.h"],
    deps = [
        "//muan/utils:math_utils",
        "//third_party/eigen",
    ],
)

cc_library(
    name = "pid_controller",
    srcs = [
        "pid_controller.cpp",
    ],
    hdrs = [
        "average_filter_pid.h",
        "pid_controller.h",
    ],
    linkstatic = 1,
    deps = [
        "//muan/units",
        "//muan/utils:history",
    ],
)

cc_library(
    name = "motion_profiles",
    srcs = [
        "trapezoidal_motion_profile.cpp",
    ],
    hdrs = [
        "motion_profile.h",
        "trapezoidal_motion_profile.h",
    ],
    linkopts = ["-lm"],
    linkstatic = 1,
    deps = [
        "//muan/units",
        "//muan/utils:math_utils",
    ],
)

cc_library(
    name = "nonlinear_feedback",
    srcs = [
        "nonlinear_feedback_controller.cpp",
    ],
    hdrs = [
        "nonlinear_feedback_controller.h",
    ],
    deps = [
        ":drivetrain_model",
        ":pose",
        "//third_party/eigen",
    ],
)

cc_library(
    name = "state_space",
    srcs = [
        "state_space_controller.hpp",
        "state_space_observer.hpp",
        "state_space_plant.hpp",
    ],
    hdrs = [
        "state_space_controller.h",
        "state_space_observer.h",
        "state_space_plant.h",
    ],
    linkopts = ["-lm"],
    linkstatic = 1,
    deps = [
        "//muan/units",
        "//muan/utils:math_utils",
        "//third_party/eigen",
    ],
)

cc_library(
    name = "trajectory",
    srcs = [
        "trajectory.cpp",
    ],
    hdrs = [
        "trajectory.h",
    ],
    deps = [
        ":drivetrain_model",
        ":spline",
    ],
)

cc_test(
    name = "pose_test",
    srcs = ["pose_test.cpp"],
    deps = [
        ":pose",
        "//third_party/google/googletest",
    ],
)

cc_test(
    name = "drivetrain_model_test",
    srcs = ["drivetrain_model_test.cpp"],
    deps = [
        ":drivetrain_model",
        "//third_party/google/googletest",
    ],
)

cc_test(
    name = "motion_profile_test",
    srcs = ["motion_profile_test.cpp"],
    deps = [
        ":motion_profiles",
        "//muan/utils:math_utils",
        "//third_party/google/googletest",
    ],
)

cc_test(
    name = "nonlinear_feedback_test",
    srcs = ["nonlinear_feedback_controller_test.cpp"],
    deps = [
        ":nonlinear_feedback",
        ":trajectory",
        "//third_party/google/googletest",
    ],
)

cc_test(
    name = "state_space_test",
    srcs = ["state_space_test.cpp"],
    deps = [
        ":state_space",
        "//third_party/google/googletest",
    ],
)

cc_test(
    name = "spline_test",
    srcs = ["spline_test.cpp"],
    deps = [
        ":spline",
        "//third_party/google/googletest",
    ],
)

cc_test(
    name = "trajectory_test",
    srcs = ["trajectory_test.cpp"],
    deps = [
        ":trajectory",
        "//third_party/google/googletest",
    ],
)

cc_test(
    name = "pid_test",
    srcs = ["pid_test.cpp"],
    deps = [
        ":pid_controller",
        "//third_party/google/googletest",
    ],
)

py_binary(
    name = "cpp_test_generator",
    srcs = ["cpp_generator_test.py"],
    main = "cpp_generator_test.py",
    deps = ["python_controls"],
)

genrule(
    name = "cpp_test_genrule",
    srcs = [],
    outs = [
        "cpp_test_constants.h",
        "cpp_test_constants.cpp",
    ],
    cmd = "$(location :cpp_test_generator) $(location cpp_test_constants.h) $(location cpp_test_constants.cpp)",
    tools = [":cpp_test_generator"],
)

cc_test(
    name = "generated_cpp_test",
    srcs = [
        "cpp_test_constants.cpp",
        "cpp_test_constants.h",
        "generated_cpp_test.cpp",
    ],
    deps = [
        ":state_space",
        "//third_party/google/googletest",
    ],
)

cc_library(
    name = "ramping",
    srcs = ["ramping.cpp"],
    hdrs = ["ramping.h"],
    linkopts = ["-lm"],
)

cc_test(
    name = "ramping_test",
    srcs = ["ramping_test.cpp"],
    deps = [
        ":ramping",
        "//third_party/google/googletest",
    ],
)
