# Build leemonscript-transpiler with bazel is only supported in the robot-code
# repo.
package(default_visibility = ['//visibility:public'])
licenses(['notice'])

load(':lemonscript.bzl', 'cc_lemonscript_library')

py_binary(
  name = 'lemonscript_transpiler',
  srcs = ['transpiler.py'] + glob(['objects/*.py']),
  main = 'transpiler.py'
)

cc_lemonscript_library(
  name = 'example_func',
  srcs = ['tests/files/transpiler/auto_functions/example.func']
)
