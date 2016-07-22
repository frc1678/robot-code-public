package(default_visibility = ['//visibility:public'])

# TODO(Kyle) Until I find a workaround, these python rules have to be run with --force_python=PY3
py_library(
  name = 'python_controls',
  srcs = [
    'controls.py',
    'state_space_controller.py',
    'state_space_observer.py',
    'state_space_plant.py',
    'state_space_scenario.py',
    'trapezoidal_profile.py'
  ],
  srcs_version = 'PY3ONLY'
)

py_binary(
  name = 'python_controls_demo',
  srcs = ['state_space_demo.py'],
  main = 'state_space_demo.py',
  srcs_version = 'PY3ONLY',
  default_python_version = 'PY3',
  deps = [':python_controls']
)

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
  ],
  srcs = [
    'trapezoidal_motion_profile.hpp'
  ],
  deps = ['@third_party//unitscpp'],
  linkopts = ['-lm'],
  linkstatic = 1
)

cc_library(
  name = 'state_space',
  hdrs = [
    'state_space_plant.h',
    'state_space_observer.h',
    'state_space_controller.h',
  ],
  srcs = [
    'state_space_plant.hpp',
    'state_space_observer.hpp',
    'state_space_controller.hpp',
  ],
  deps = [
    '@third_party//unitscpp',
    '@third_party//eigen',
    '//utils:math_utils'
  ],
  linkopts = ['-lm'],
  linkstatic = 1
)

cc_test(
  name = 'motion_profile_test',
  srcs = ['motion_profile_test.cpp'],
  deps = [
    ':motion_profiles',
    '//utils:math_utils',
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
