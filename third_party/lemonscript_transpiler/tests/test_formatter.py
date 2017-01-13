import os
import sys
import shutil

parent_dir = os.path.dirname(os.path.realpath(__file__)) + "/../"
sys.path.append(parent_dir) # a bit of a hack, but it makes the import the same
from objects.formatter import Formatter

def get_file_path(path):
    return os.path.dirname(os.path.realpath(__file__)) + "/" + path

class TestFormatterMethods():

    def test_get_clang_path_exists(self):
        formatter = Formatter("test")

        clang_path_suffixes = ["", "-3.7", "-3.6", "-3.5"]

        for path_suffix in clang_path_suffixes:
            try:
                shutil.rmtree("/tmp/test_root/")
            except OSError:
                pass

            os.mkdir("/tmp/test_root/")
            os.mkdir("/tmp/test_root/usr/")
            os.mkdir("/tmp/test_root/usr/bin/")
            open("/tmp/test_root/usr/bin/clang-format" + path_suffix, "w").close()

            assert formatter.get_clang_path("/tmp/test_root") == "/usr/bin/clang-format" + path_suffix

    def test_clang_path_doesnt_exist(self):
        formatter = Formatter("test")

        try:
            shutil.rmtree("/tmp/test_root/")
        except OSError:
            pass

        os.mkdir("/tmp/test_root/")
        os.mkdir("/tmp/test_root/usr/")
        os.mkdir("/tmp/test_root/usr/bin/")

        assert formatter.get_clang_path("/tmp/test_root") == None

    def test_get_formatted_text(self):
        formatter = Formatter("test")

        assert "test_passed" in formatter.get_formatted_text(get_file_path("files/formatter/mock_clang_format.py"))
        assert "-style=" in formatter.get_formatted_text(get_file_path("files/formatter/mock_clang_format.py"))

    def test_get_formatted_text_clang_format_failure(self):
        formatter_input = "test_fail"
        formatter = Formatter(formatter_input)

        assert formatter.get_formatted_text(get_file_path("files/formatter/mock_clang_format_fail.sh")) == formatter_input

    def test_get_formatted_text_style(self):
        formatter = Formatter("test", style="test_style_passed")

        assert "-style=test_style_passed" in formatter.get_formatted_text(get_file_path("files/formatter/mock_clang_format.py"))
