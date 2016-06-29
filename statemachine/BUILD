package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'statemachine',
  hdrs = [
    'state.h',
    'state_machine.h'
  ],
  srcs = [
    'state.cpp',
    'state_machine.cpp'
  ]
)

cc_test(
  name = 'statemachine_test',
  srcs = ['state_machine_test.cpp'],
  deps = [
    ':statemachine',
    '@third_party//google/googletest:googletest'
  ]
)
