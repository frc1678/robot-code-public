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
    name = "state_space_test",
    srcs = ["state_space_test.cpp"],
    deps = [
        ":state_space",
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
