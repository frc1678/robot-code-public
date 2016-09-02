#!/usr/bin/python3
import glob
import subprocess as sp

sp.call(['bazel', 'build', '--experimental_action_listener=//tools:compilation_database', '//...', '--', '-//third_party/...'])
files = glob.iglob("bazel-out/local_linux-fastbuild/extra_actions/tools/cpp_compilation_commands_gatherer/**/*.json", recursive=True)

json_arr = []

for name in files:
    with open(name) as json:
        json_arr.append(json.read())

with open('compile_commands.json', 'w') as output:
    output.write('[' + ','.join(json_arr) + ']')
