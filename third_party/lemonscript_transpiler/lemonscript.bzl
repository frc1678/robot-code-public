def cc_lemonscript_library(
    name,
    srcs,
    deps = [],
    transpiler = '//third_party/lemonscript_transpiler',
    lemonscript = '//third_party/lemonscript'):

  native.genrule(
    name = name + '_genrule',
    srcs = srcs,
    tools = [transpiler],
    cmd = '$(location %s) --input-files %s --output-cpp $(location %s) --output-header $(location %s)' % (
        transpiler,
        ' '.join(['$(location %s)' % s for s in srcs]),
        name + '.cpp',
        name + '.h'),
    outs = [name + '.cpp', name + '.h'],
  )

  native.cc_library(
    name = name,
    srcs = [name + '.cpp'],
    hdrs = [name + '.h'],
    deps = [
      lemonscript,
      '//muan/units',
    ] + deps,
)
