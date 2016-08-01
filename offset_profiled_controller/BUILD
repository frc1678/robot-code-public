package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'calibration',
  hdrs = [
    'hall_calibration.h'
  ],
  srcs = [
    'hall_calibration.cpp'
  ],
  linkstatic = 1
)

cc_test(
  name = 'calibration_test',
  srcs = [
    'hall_calibration_test.cpp'
  ],
  deps = [
    ':calibration',
    '@third_party//google/googletest:googletest'
  ]
)
