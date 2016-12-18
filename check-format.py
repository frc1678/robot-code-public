#!/usr/bin/python

import subprocess as sp
import os
import sys
import re
import distutils.spawn

# Allow this script to not cause CI checks to fail
if "--no-fail" in sys.argv:
    failcode = 0
else:
    failcode = 1

p = sp.Popen('git ls-files', shell=True, stdin=sp.PIPE, stdout=sp.PIPE, stderr=sp.STDOUT, close_fds=True)

files = p.stdout.read().split('\n')[0:-1]

cpp_regex = re.compile("\\.(cpp|c|hpp|h)$")
thirdparty_regex = re.compile("third.party")

to_format = []

for f in files:
    if not thirdparty_regex.search(f) and cpp_regex.search(f):
        to_format.append(f)

clang_format_command = ''
supported_clang_format_commands = [
        'clang-format-3.5',
        'clang-format-3.6',
        'clang-format-3.7',
        'clang-format-3.8',
        'clang-format'
        ]
for com in supported_clang_format_commands:
    if distutils.spawn.find_executable(com):
        clang_format_command = com

for f in to_format:
    command = '%s %s | diff - %s' % (clang_format_command, f, f)
    p = sp.Popen(command, shell=True, stdin=sp.PIPE, stdout=sp.PIPE, stderr=sp.STDOUT, close_fds=True)
    s = p.stdout.read()
    if len(s) > 0:
        print("Formatting error in file %s" % f)
        print(s)
        exit(failcode)

exit(0)
