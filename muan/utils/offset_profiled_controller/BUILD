cc_library(
  name = 'calibration',
  hdrs = [
    'hall_calibration.h',
    'pot_calibration.h'
  ],
  srcs = [
    'hall_calibration.cpp',
    'pot_calibration.cpp'
  ],
  visibility = ['//visibility:public']
)

cc_test(
  name = 'calibration_test',
  srcs = [
    'hall_calibration_test.cpp',
    'pot_calibration_test.cpp'
  ],
  deps = [
    ':calibration',
    '//third_party/google/googletest:googletest',
    '//muan/utils:math_utils'
  ],
  linkopts = ['-lm']
)
