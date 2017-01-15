genrule(
  name = 'lemonscript_generate',
  srcs = [
    'funcs/test.func'
  ],
  outs = [
    'ls_gen.cpp',
    'ls_gen.h'
  ],
  tools = [
    '//third_party/lemonscript_transpiler:lemonscript_transpiler'
  ],
  cmd = "$(location //third_party/lemonscript_transpiler:lemonscript_transpiler) --input-files $(location funcs/test.func) --output-cpp $(location ls_gen.cpp) --output-header $(location ls_gen.h)",
)

cc_library(
  name = 'lemonscript_funcs_cpp',
  srcs = [
    'ls_gen.cpp',
  ],
  hdrs = [
    'ls_gen.h'
  ],
  deps = [
    '//muan/actions:drivetrain_action',
    '//c2017/subsystems/drivetrain:drivetrain_base',
    '//c2017/queue_manager:queue_manager',
    '//third_party/lemonscript:lemonscript'
  ]
)

cc_library(
  name = 'lemonscript',
  srcs = [
    'lemonscript.h',
    'lemonscript.cpp',
  ],
  deps = [
    ':lemonscript_funcs_cpp',
    '//muan/actions:drivetrain_action',
    '//c2017/subsystems/drivetrain:drivetrain_base',
    '//c2017/queue_manager:queue_manager',
    '//third_party/lemonscript:lemonscript',
    '//third_party/aos/linux_code:init',
    '//third_party/aos/common/util:phased_loop',
    '//third_party/aos/common:time'
  ],
  visibility = ['//visibility:public']
)

cc_binary(
  name = 'test',
  srcs = ['test.cpp'],
  deps = [':lemonscript']
)
