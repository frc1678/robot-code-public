licenses(["permissive"])

cc_library(
  name = 'unitscpp',
  hdrs = [':unitscpp_genrule'],
  visibility = ['//visibility:public'],
  linkstatic = 1
)

genrule(
  name = 'unitscpp_genrule',
  srcs = ['units.cfg'],
  tools = [':unitscpp_generator'],
  cmd = '$(location :unitscpp_generator) $(location :units.cfg) > $(location unitscpp.h)',
  outs = ['unitscpp.h']
)

py_binary(
  name = 'unitscpp_generator',
  srcs = ['gen.py'],
  srcs_version = 'PY2AND3',
  default_python_version = 'PY3',
  main = 'gen.py'
)
