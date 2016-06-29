package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'pid_controller',
  hdrs = [
    'pid_controller.h',
    'average_filter_pid.h'
  ],
  deps = [
    '@third_party//unitscpp',
    '//utils:history'
  ],
  linkstatic = 1
)

cc_library(
  name = 'motion_profiles',
  hdrs = [
    'motion_profile.h',
    'trapezoidal_motion_profile.h',
    'linear_motion_profile.h',
    'instant_motion_profile.h'
  ],
  deps = ['@third_party//unitscpp'],
  linkopts = ['-lm'],
  linkstatic = 1
)

cc_library(
  name = 'state_space',
  hdrs = [
    'state_space_plant.h',
    'state_observer.h',
    'state_feedback_controller.h',
    'control_utils.h'
  ],
  deps = [
    '@third_party//unitscpp',
    '@third_party//eigen'
  ],
  linkopts = ['-lm'],
  linkstatic = 1
)

cc_test(
  name = 'motion_profile_test',
  srcs = ['motion_profile_test.cpp'],
  deps = [
    ':motion_profiles',
    '@third_party//google/googletest:googletest'
  ]
)

cc_test(
  name = 'state_space_test',
  srcs = ['state_space_test.cpp'],
  deps = [
    ':state_space',
    '@third_party//google/googletest:googletest'
  ]
)

cc_test(
  name = 'pid_test',
  srcs = ['pid_test.cpp'],
  deps = [
    ':pid_controller',
    '@third_party//google/googletest:googletest'
  ]
)
