import numpy as np
from functools import reduce
from copy import copy

"""
A class to handle writing state-space gains to files.
"""
__author__ = 'Kyle Stachowicz (kylestach99@gmail.com)'

class StateSpaceWriter(object):
    def __init__(self, gains, name):
        if not isinstance(gains, list):
            gains = [gains]
        self.gains = gains

        self.namespaces = ['frc1678', name, 'controller']

    # Write C++ code to open all namespaces
    def _namespace_openers(self):
        return '\n'.join(['namespace {} {{'.format(ns) for ns in self.namespaces])

    # Write the curly braces to close all namespaces
    def _namespace_closers(self):
        return '\n'.join(['}} /* {} */'.format(ns) for ns in reversed(self.namespaces)])

    # Write the declaration for a single C++ matrix generator
    def _matrix_generator_declaration(self, name, mat):
        format_string = 'Eigen::Matrix<double, {n}, {m}> {name}();'
        return format_string.format(n = mat.shape[0], m = mat.shape[1], name = name)

    # Write all C++ matrix generator declarations
    def _matrix_generator_declarations(self, writable_matrices):
        return '\n'.join(
            [self._matrix_generator_declaration(name, writable_matrices[name]) for name in writable_matrices]
        )

    # Write a single C++ matrix generator
    def _matrix_generator_definition(self, name, mat):
        entries = ', '.join([str(x) for x in np.nditer(mat)])
        format_string = \
'''Eigen::Matrix<double, {n}, {m}> {name}() {{
    return (Eigen::Matrix<double, {n}, {m}>() << {entries}).finished();
}}'''
        return format_string.format(n = mat.shape[0], m = mat.shape[1], name = name, entries = entries)

    # Write all C++ matrix generators
    def _matrix_generator_definitions(self, writable_matrices):
        return '\n'.join(
            [self._matrix_generator_definition(name, writable_matrices[name]) for name in writable_matrices]
        )

    def _gain_generator_declarations(self):
        out_string = ''
        for gain in self.gains:
            if len(self.gains) != 1:
                out_string += 'namespace %s {\n' % gain.name

            out_string += self._matrix_generator_declarations(gain.writable_matrices())

            if len(self.gains) != 1:
                out_string += '\n} // namespace %s\n' % gain.name
        return out_string

    def _gain_generator_definitions(self):
        out_string = ''
        for gain in self.gains:
            if len(self.gains) != 1:
                out_string += 'namespace %s {\n' % gain.name

            out_string += self._matrix_generator_definitions(gain.writable_matrices())

            if len(self.gains) != 1:
                out_string += '\n} // namespace %s\n' % gain.name

        return out_string

    # Write C++ code to a header file and a cpp file
    def write(self, h_file_name, cpp_file_name):
        with open(h_file_name, 'w') as h_file, open(cpp_file_name, 'w') as cpp_file:
            h_file.write(
'''#pragma once
#include "Eigen/Core"

{namespace_openers}
{gain_generators}
{namespace_closers}
'''.format(namespace_openers = self._namespace_openers(), gain_generators = self._gain_generator_declarations(), namespace_closers = self._namespace_closers()))

            cpp_file.write(
'''#include "{header_name}"
{namespace_openers}
{gain_generators}
{namespace_closers}
'''.format(header_name = h_file_name, namespace_openers = self._namespace_openers(), gain_generators = self._gain_generator_definitions(), namespace_closers = self._namespace_closers()))
