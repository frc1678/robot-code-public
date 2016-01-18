cc_test(
  name = "multithreading",
  srcs = ["multithreading_test.cpp"],
  deps = ["//:multithreading", "@gtest//:main"]
)

cc_test(
  name = "statemachine",
  deps = ["//:statemachine", "@gtest//:main"],
  srcs = ["state_machine_test.cpp"]
)

cc_test(
  name = "utils",
  deps = ["//:utils", "@gtest//:main"],
  srcs = ["utils_test.cpp"]
)

cc_test(
  name = "pid",
  deps = ["//:utils", "@gtest//:main"],
  srcs = ["pid_test.cpp"]
)

cc_test(
  name = "motionprofile",
  deps = ["//:control", "@gtest//:main"],
  srcs = ["motion_profile_test.cpp"],
  linkopts = ["-lm"]
)

cc_test(
  name = "state_space",
  deps = ["//:control", "@gtest//:main", "@eigen//:main"],
  srcs = ["state_space_test.cpp"],
  linkopts = ["-lm"]
)

cc_test(
  name = "dummy_system",
  deps = ["//:muan", "@gtest//:main"],
  srcs = glob(["dummy_system/**/*.cpp", "dummy_system/**/*.h"]),
)
