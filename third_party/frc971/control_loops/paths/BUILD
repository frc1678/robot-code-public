licenses(['notice'])
package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'path',
  srcs = ['path.cc'],
  hdrs = ['path.h'],
  deps = ['//third_party/eigen'],
)

cc_test(
  name = 'path_test',
  srcs = ['path_test.cc'],
  deps = [
    ':path',
    '//third_party/aos/testing:googletest',
  ],
)

cc_library(
  name = 'trajectory',
  srcs = ['trajectory.cc'],
  hdrs = ['trajectory.h'],
  deps = [
    ':path',
    '//third_party/eigen',
    '//third_party/aos/common:die',
    '//muan/logging:logger',
  ],
)

cc_test(
  name = 'trajectory_test',
  srcs = ['trajectory_test.cc'],
  deps = [
    ':trajectory',
    '//third_party/aos/testing:googletest',
  ],
)
