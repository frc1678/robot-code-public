import third_party.google.bazel.src.main.protobuf.extra_actions_pb2 as extra_actions
import os
import sys

invalid_clang_flags = [
    '-fno-canonical-system-headers'
]

def cppinfo_to_json(source_file, compile_info):
    out_fmt = '{{"directory":"{pwd}","command":"{command}","file":"{sfile}"}}'
    pwd = os.getcwd()

    options = ' '.join(filter(lambda opt: not opt in invalid_clang_flags, compile_info.compiler_option))
    command = '%s %s -x c++ -Wno-unknown-warning-option -c %s -o %s' % (compile_info.tool, options, compile_info.source_file, compile_info.output_file)

    return out_fmt.format(pwd = pwd, command = command.replace('"', '\\"'), sfile = source_file)


with open(sys.argv[1]) as in_proto:
    compile_info = extra_actions.ExtraActionInfo()
    compile_info.ParseFromString(in_proto.read())

    cpp_compile_info = compile_info.Extensions[extra_actions.CppCompileInfo.cpp_compile_info]

    descriptors = []
    for source in cpp_compile_info.sources_and_headers:
        descriptors.append(cppinfo_to_json(source, cpp_compile_info))
    with open(sys.argv[2], 'w') as out_json:
        out_json.write(','.join(descriptors))
