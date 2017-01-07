package(default_visibility = ['//visibility:public'])
licenses(['notice'])

cc_library(
  name = 'polytope',
  hdrs = [
    'polytope.h',
  ],
  deps = [
    '//third_party/eigen',
    '//third_party/cddlib',
    '//third_party/aos/common:check',
  ],
)

cc_test(
  name = 'polytope_test',
  srcs = [
    'polytope_test.cc',
  ],
  deps = [
    ':polytope',
    '//third_party/aos/testing:googletest',
    '//third_party/eigen',
    '//third_party/google/googletest:googlemock',
  ],
)
