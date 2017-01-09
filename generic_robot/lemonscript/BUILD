cc_library(
  name = 'lemonscript',
  srcs = [
    'lemonscript.h',
    'lemonscript.cpp'
  ],
  deps = [
    '//third_party/lemonscript:lemonscript',
    '//third_party/lemonscript_transpiler:example_func',
    '//third_party/aos/linux_code:init',
    '//third_party/aos/common/util:phased_loop',
    '//third_party/aos/common:time'
  ]
)

cc_binary(
  name = 'test',
  srcs = ['test.cpp'],
  deps = [':lemonscript']
)
