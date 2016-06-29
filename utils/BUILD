package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'math_utils',
  hdrs = ['math_utils.h'],
  deps = ["@third_party//unitscpp"],
  linkstatic = 1
)

cc_library(
  name = 'history',
  hdrs = ['history.h'],
  deps = ["@third_party//unitscpp"],
  linkstatic = 1
)

cc_library(
  name = 'timing',
  hdrs = [
    'timing_utils.h',
    'timer.h'
  ],
  srcs = [
    'timing_utils.cpp',
    'timer.cpp'
  ],
  deps = ["@third_party//unitscpp"]
)

cc_test(
  name = 'utils_test',
  srcs = ['utils_test.cpp'],
  deps = [
    ':timing',
    'history',
    '@third_party//google/googletest:googletest'
  ]
)
