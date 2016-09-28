import numpy as np
from functools import reduce
from copy import copy

"""
A class to handle writing state-space gains to files.
"""
__author__ = 'Kyle Stachowicz (kylestach99@gmail.com)'

# Generic C++ code generation helpers
def _namespace_openers(namespaces):
    return '\n'.join(['namespace {} {{'.format(ns) for ns in namespaces])

def _namespace_closers(namespaces):
    return '\n'.join(['}} /* {} */'.format(ns) for ns in reversed(namespaces)])

def _matrix_generator_declaration(name, mat):
    format_string = 'Eigen::Matrix<double, {n}, {m}> {name}();'
    return format_string.format(n = mat.shape[0], m = mat.shape[1], name = name)

def _matrix_generator_definition(name, mat):
    # Use C-order to traverse the matrix
    entries = ', '.join([str(x) for x in np.nditer(mat, order='C')])
    format_string = \
'''Eigen::Matrix<double, {n}, {m}> {name}() {{
    return (Eigen::Matrix<double, {n}, {m}>() << {entries}).finished();
}}'''
    return format_string.format(
        n = mat.shape[0],
        m = mat.shape[1],
        name = name,
        entries = entries
    )

def _generator_declaration_for(name, constant):
    if isinstance(constant, np.matrix):
        return _matrix_generator_declaration(name, constant)
    elif isinstance(constant, float):
        return 'double {}();'.format(name)
    else:
        raise ValueError("Unsupported constant type {}".format(type(constant)))

def _generator_definition_for(name, constant):
    if isinstance(constant, np.matrix):
        return _matrix_generator_definition(name, constant)
    elif isinstance(constant, float):
        return 'double {}() {{ return {}; }}'.format(name, constant)
    else:
        raise ValueError("Unsupported constant type {}".format(type(constant)))

def _all_generator_declarations(constants):
    return '\n'.join([
        _generator_declaration_for(name, constant) for name, constant in list(constants.items())
    ])

def _all_generator_definitions(constants):
    return '\n'.join([
        _generator_definition_for(name, constant) for name, constant in list(constants.items())
    ])

class StateSpaceWriter(object):
    def __init__(self, gains, name):
        if not isinstance(gains, list):
            gains = [gains]
        self.gains = gains

        self.namespaces = ['frc1678', name, 'controller']

    def _gain_generator_declarations(self):
        out_string = ''
        for gain in self.gains:
            if len(self.gains) != 1:
                out_string += 'namespace %s {\n' % gain.name

            out_string += _all_generator_declarations(gain.writable_constants())

            if len(self.gains) != 1:
                out_string += '\n} // namespace %s\n' % gain.name
        return out_string

    def _gain_generator_definitions(self):
        out_string = ''
        for gain in self.gains:
            if len(self.gains) != 1:
                out_string += 'namespace %s {\n' % gain.name

            out_string += _all_generator_definitions(gain.writable_constants())

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
'''.format(
        namespace_openers = _namespace_openers(self.namespaces),
        gain_generators = self._gain_generator_declarations(),
        namespace_closers = _namespace_closers(self.namespaces)
    ))

            cpp_file.write(
'''#include "{header_name}"
{namespace_openers}
{gain_generators}
{namespace_closers}
'''.format(
        header_name = h_file_name,
        namespace_openers = _namespace_openers(self.namespaces),
        gain_generators = self._gain_generator_definitions(),
        namespace_closers = _namespace_closers(self.namespaces)
    ))
