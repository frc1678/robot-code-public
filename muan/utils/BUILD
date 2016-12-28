package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'math_utils',
  hdrs = ['math_utils.h'],
  srcs = ['math_utils.cpp'],
  deps = [
    '//muan/units:units',
    '//third_party/eigen'
  ],
  linkopts = ['-lm'],
  linkstatic = 1
)

cc_library(
  name = 'history',
  hdrs = ['history.h'],
  deps = [
    '//muan/units:units',
    '//third_party/aos/common:die'
  ],
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
  deps = ['//muan/units:units']
)

cc_library(
  name = 'linear_interpolation',
  hdrs = ['linear_interpolation.h'],
  srcs = ['linear_interpolation.hpp'],
  deps = ['//third_party/aos/common:die']
)

cc_test(
  name = 'linear_interpolation_test',
  srcs = ['linear_interpolation_test.cpp'],
  deps = [
    ':linear_interpolation',
    '//third_party/google/googletest:googletest'
  ]
)

cc_test(
  name = 'history_test',
  srcs = ['history_test.cpp'],
  deps = [
    ':history',
    '//third_party/google/googletest:googletest'
  ]
)

cc_test(
  name = 'time_test',
  srcs = ['time_test.cpp'],
  deps = [
    ':timing',
    '//third_party/google/googletest:googletest'
  ]
)
