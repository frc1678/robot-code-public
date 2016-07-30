package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'calibration',
  hdrs = [
    'calibration.h',
    'hall_encoder_calibration.h'
  ],
  srcs = [
    'calibration.hpp',
    'hall_encoder_calibration.cpp'
  ],
  linkstatic = 1
)

cc_test(
  name = 'calibration_test',
  srcs = [
    'hall_encoder_test.cpp'
  ],
  deps = [
    ':calibration',
    '@third_party//google/googletest:googletest'
  ]
)
