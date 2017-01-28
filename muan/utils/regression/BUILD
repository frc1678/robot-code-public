py_library(
    name = "state_space_regression_lib",
    srcs = ["regression.py"],
    visibility = ["//visibility:public"],
)

py_binary(
    name = "state_space_regression",
    srcs = ["regression_main.py"],
    main = "regression_main.py",
)

py_test(
    name = "regression_test",
    srcs = ["regression_test.py"],
    deps = ["state_space_regression_lib"],
)
