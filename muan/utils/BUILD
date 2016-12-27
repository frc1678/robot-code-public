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
  name = 'disk_brake',
  hdrs = ['disk_brake.h'],
  srcs = ['disk_brake.cpp'],
  deps = [
    '//muan/units:units',
    ':math_utils',
    '//third_party/aos/common:time'
  ]
)

cc_library(
  name = 'linear_interpolation',
  hdrs = ['linear_interpolation.h'],
  srcs = ['linear_interpolation.hpp'],
  deps = ['//third_party/aos/common:die']
)

cc_test(
  name = 'utils_test',
  srcs = ['utils_test.cpp'],
  deps = [
    ':timing',
    ':history',
    ':linear_interpolation',
    ':disk_brake',
    '//third_party/google/googletest:googletest'
  ]
)
