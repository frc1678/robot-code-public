package(default_visibility = ['//visibility:public'])

load('/tools/select', 'cpu_select')

cc_library(
  name = 'libpthread',
  srcs = [],
  hdrs = [],
  linkopts = ['-lpthread', '-pthread']
)

[ cc_library(
  name = 'lib' + name,
  srcs = [],
  hdrs = [],
  linkopts = ['-l' + name]
  ) for name in ['m', 'dl', 'rt'] ]
