import subprocess
import os

from .logger import Logger

class Formatter(object):
    def __init__(self, input_text, style=None): # style=None is a pretty good description of me tbh
        self.text = input_text
        if style == None:
            Logger.debug("Using default style for formatting.")
            self.style = "{BasedOnStyle: Google, ColumnLimit: 0}"
        else:
            Logger.debug("Using custom style for formatting: {}".format(style))
            self.style = style

    def get_formatted_text(self, clang_path=None):
        if clang_path == None:
            clang_path = self.get_clang_path()
        if clang_path == None: # pragma: no cover
            Logger.warn("clang-format not found! Not formatting output.")
            return self.text
        try:
            clang_process = subprocess.Popen([clang_path, "-style=" + self.style], stdin=subprocess.PIPE, stdout=subprocess.PIPE)
            return clang_process.communicate(input=bytes(self.text.encode("utf-8")))[0].decode("utf-8")
        except OSError:
            Logger.warn("Error running clang-format! Not formatting output.")
            return self.text


    def get_clang_path(self, test_prefix=""):
        possible_paths = [
            "/usr/bin/clang-format",
            "/usr/bin/clang-format-3.7",
            "/usr/bin/clang-format-3.6",
            "/usr/bin/clang-format-3.5",
            "/usr/local/bin/clang-format",
            "/usr/local/bin/clang-format-3.7",
            "/usr/local/local/bin/clang-format-3.6",
            "/usr/local/local/bin/clang-format-3.5"
        ]

        for path in possible_paths:
            if os.path.isfile(test_prefix + path):
                Logger.info("Found clang formatter: {}".format(path))
                return path

        return None
