package(default_visibility = ['//visibility:public'])
licenses(['notice'])

cc_library(
  name = 'math',
  hdrs = [
    'commonmath.h',
  ],
)

cc_library(
  name = 'macros',
  hdrs = [
    'macros.h',
  ],
)

cc_library(
  name = 'type_traits',
  hdrs = [
    'type_traits.h',
  ],
)

cc_library(
  name = 'time',
  srcs = [
    'time.cc',
  ],
  hdrs = [
    'time.h',
  ],
  deps = [
    ':mutex',
    ':macros',
  ],
)

cc_library(
  name = 'unique_malloc_ptr',
  hdrs = [
    'unique_malloc_ptr.h',
  ],
)

cc_library(
  name = 'network_port',
  hdrs = [
    'network_port.h',
  ],
)

cc_library(
  name = 'byteorder',
  hdrs = [
    'byteorder.h',
  ],
)

cc_test(
  name = 'type_traits_test',
  srcs = [
    'type_traits_test.cpp',
  ],
  deps = [
    '//third_party/aos/testing:googletest',
    ':type_traits',
  ],
)

cc_library(
  name = 'gtest_prod',
  hdrs = [
    'gtest_prod.h',
  ],
)

cc_library(
  name = 'once',
  hdrs = [
    'once.h',
  ],
  srcs = [
    'once-tmpl.h',
  ],
  deps = [
    ':gtest_prod',
    ':type_traits',
  ],
)

cc_test(
  name = 'once_test',
  srcs = [
    'once_test.cc',
  ],
  deps = [
    '//third_party/aos/testing:googletest',
    ':once',
  ],
)

cc_test(
  name = 'time_test',
  srcs = [
    'time_test.cc',
  ],
  deps = [
    '//third_party/aos/testing:googletest',
    ':time',
    '//linux_libraries:libpthread',
  ],
)

cc_library(
  name = 'die',
  srcs = [
    'die.cc',
  ],
  hdrs = [
    'die.h',
  ],
  deps = [
    ':macros',
    '//third_party/aos/common/libc:aos_strerror',
  ],
)

cc_library(
  name = 'check',
  srcs = [
  ],
  hdrs = [
    'check.h',
  ],
  deps = [
    ':die',
  ],
)

cc_test(
  name = 'mutex_test',
  srcs = [
    'mutex_test.cc',
  ],
  deps = [
    '//third_party/aos/testing:googletest',
    ':mutex',
    ':die',
    '//third_party/aos/common/util:thread',
    '//third_party/aos/common:time',
  ],
)

# TODO(Kyle): If we ever want to use condition variables, figure out why this test hangs
"""
cc_library(
  name = 'condition',
  hdrs = [
    'condition.h',
  ],
  srcs = [
    'condition.cc',
  ],
  deps = [
    ':mutex',
    ':check',
    '//third_party/aos/linux_code/ipc_lib:aos_sync',
  ],
)

cc_test(
  name = 'condition_test',
  srcs = [
    'condition_test.cc',
  ],
  deps = [
    '//third_party/aos/testing:googletest',
    '//third_party/aos/testing:prevent_exit',
    ':condition',
    '//third_party/aos/common/util:thread',
    ':time',
    ':mutex',
    '//third_party/aos/linux_code/ipc_lib:aos_sync',
    ':die',
  ],
)
"""

cc_test(
  name = 'die_test',
  srcs = [
    'die_test.cc',
  ],
  deps = [
    '//third_party/aos/testing:googletest',
    ':die',
  ],
)

cc_library(
  name = 'stl_mutex',
  hdrs = [
    'stl_mutex.h',
  ],
  deps = [
    '//third_party/aos/linux_code/ipc_lib:aos_sync',
    ':die',
    ':check',
    '//linux_libraries:libpthread',
  ],
)

cc_library(
  name = 'mutex',
  hdrs = [
    'mutex.h',
  ],
  srcs = [
    'mutex.cc',
  ],
  deps = [
    '//third_party/aos/linux_code/ipc_lib:aos_sync',
    ':die',
    ':check',
    ':type_traits',
    '//linux_libraries:libpthread',
  ],
)

cc_test(
  name = 'stl_mutex_test',
  srcs = [
    'stl_mutex_test.cc',
  ],
  deps = [
    ':stl_mutex',
    '//third_party/aos/testing:googletest',
    '//third_party/aos/common/util:thread',
    ':die',
  ],
)
