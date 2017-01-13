# All of the toolchains use the same layout, so we can just generate them with
# a big for-loop macro
def generate_cc_toolchain(cpu):
  native.cc_toolchain(
      name = ("cc-compiler-%s" % cpu),
      all_files = ":empty",
      compiler_files = ":empty",
      cpu = cpu,
      dwp_files = ":empty",
      dynamic_runtime_libs = [":empty"],
      linker_files = ":empty",
      objcopy_files = ":empty",
      static_runtime_libs = [":empty"],
      strip_files = ":empty",
      supports_param_files = 0,
  )

def generate_cc_toolchains(cpus):
  for cpu in cpus:
    generate_cc_toolchain(cpu)

# The keys of the toolchains dictionary are obtained via {cpu}|{compiler} in
# bazel >=0.3.2, but in <0.3.1 it's just {cpu}. Keep two copies of all of the
# entries to compensate for that, at least until we move to 0.3.2.
def generate_cc_toolchain_suite(name, cpus, compilers = ['gcc', 'clang']):
  toolchain_dict = {}
  for cpu in cpus:
    toolchain_dict[cpu] = ':cc-compiler-%s' % cpu
    toolchain_dict['%s|compiler' % cpu] = ':cc-compiler-%s' % cpu
    for compiler in compilers:
      toolchain_dict['%s|%s' % (cpu, compiler)] = ':cc-compiler-%s' % cpu

  native.cc_toolchain_suite(
    name = name,
    toolchains = toolchain_dict,
  )
