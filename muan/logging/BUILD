package(default_visibility = ['//visibility:public'])

cc_library(
  name = 'log_base',
  hdrs = ['log_manager.h', 'log.h'],
  srcs = ['log_manager.cpp', 'log.cpp'],
  deps = ['//utils:timing'],
  visibility = ['//visibility:private']
)

cc_library(
  name = 'text_log',
  hdrs = ['text_log.h'],
  srcs = ['text_log.cpp'],
  deps = [':log_base']
)

cc_library(
  name = 'csv_log',
  hdrs = ['csv_log.h'],
  srcs = ['csv_log.cpp'],
  deps = [':log_base']
)

cc_library(
  name = 'test_log',
  hdrs = ['test_log.h'],
  srcs = ['test_log.cpp'],
  deps = [':log_base']
)
