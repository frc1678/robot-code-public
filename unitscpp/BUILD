cc_library(
  name = 'unitscpp',
  srcs = [],
  hdrs = [':units_autogen'],
  visibility = ["//visibility:public"],
)

genrule(
  name = 'units_autogen',
  srcs = ['units.cfg', 'gen.py'],
  cmd = 'python2 muan/unitscpp/gen.py muan/unitscpp/units.cfg > $@',
  outs = ['unitscpp.h']
)
