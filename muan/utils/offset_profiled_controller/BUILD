cc_library(
  name = 'hall_calibration',
  hdrs = ['hall_calibration.h'],
  srcs = ['hall_calibration.cpp'],
  visibility = ['//visibility:public']
)

cc_library(
  name = 'pot_calibration',
  hdrs = ['pot_calibration.h'],
  srcs = ['pot_calibration.cpp'],
  visibility = ['//visibility:public']
)

cc_test(
  name = 'hall_calibration_test',
  srcs = [
    'hall_calibration_test.cpp',
  ],
  deps = [
    ':hall_calibration',
    '//third_party/google/googletest:googletest',
    '//muan/utils:math_utils'
  ],
  linkopts = ['-lm']
)

cc_test(
  name = 'pot_calibration_test',
  srcs = [
    'pot_calibration_test.cpp'
    ],
  deps = [
    ':pot_calibration',
    '//third_party/google/googletest:googletest',
    '//muan/utils:math_utils'
  ],
  linkopts = ['-lm']
)
