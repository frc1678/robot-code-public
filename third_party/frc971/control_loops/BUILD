package(default_visibility = ['//visibility:public'])
licenses(['notice'])

cc_test(
  name = 'state_feedback_loop_test',
  srcs = [
    'state_feedback_loop_test.cc',
  ],
  deps = [
    ':state_feedback_loop',
    '//third_party/aos/testing:googletest',
  ],
)

cc_library(
  name = 'gaussian_noise',
  srcs = [
    'gaussian_noise.cc',
  ],
  hdrs = [
    'gaussian_noise.h',
  ],
  deps = [
    '//linux_libraries:libm',
  ],
)

cc_library(
  name = 'coerce_goal',
  srcs = [
    'coerce_goal.cc',
  ],
  hdrs = [
    'coerce_goal.h',
  ],
  deps = [
    '//third_party/eigen',
    '//third_party/aos/common/controls:polytope',
    '//linux_libraries:libm',
  ],
)

cc_library(
  name = 'state_feedback_loop',
  hdrs = [
    'state_feedback_loop.h',
  ],
  deps = [
    '//third_party/eigen',
    # '//third_party/aos/common/logging',
    '//third_party/aos/common:macros',
  ],
)

cc_library(
  name = 'simple_capped_state_feedback_loop',
  hdrs = [
    'simple_capped_state_feedback_loop.h',
  ],
  deps = [
    '//third_party/eigen',
    ':state_feedback_loop',
  ],
)
