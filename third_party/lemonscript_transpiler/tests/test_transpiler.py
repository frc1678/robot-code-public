import os
import subprocess
import shutil

import transpiler

class TestTranspiler:

    def clean_auto_funcs(self):
        try:
            shutil.rmtree("/tmp/auto_files")
            os.remove("/tmp/auto_functions.cpp")
            os.remove("/tmp/auto_functions.h")
            os.remove("auto_functions.cpp")
            os.remove("auto_functions.h")
        except OSError:
            pass


    def test_transpiler_creates_files_without_format(self):
        self.clean_auto_funcs()

        transpiler.main(["--output-cpp", "/tmp/auto_functions.cpp", "--output-header", "/tmp/auto_functions.h"])

        assert os.path.isfile("/tmp/auto_functions.cpp")
        assert os.path.isfile("/tmp/auto_functions.h")

    def test_transpiler_creates_files_with_format(self):
        self.clean_auto_funcs()

        transpiler.main(["--format", "--output-cpp", "/tmp/auto_functions.cpp", "--output-header", "/tmp/auto_functions.h"])

        assert os.path.isfile("/tmp/auto_functions.cpp")
        assert os.path.isfile("/tmp/auto_functions.h")

    def test_transpiler_uses_input_files(self):
        self.clean_auto_funcs()

        transpiler.main(["--format", "--output-cpp", "/tmp/auto_functions.cpp", "--output-header", "/tmp/auto_functions.h", "--input-files", "tests/files/transpiler/auto_functions/wait.func"])

        assert os.path.isfile("/tmp/auto_functions.cpp")
        assert os.path.isfile("/tmp/auto_functions.h")
        assert "Wait" in open("/tmp/auto_functions.h").read()
        assert "Wait" in open("/tmp/auto_functions.cpp").read()

    def test_transpiler_works_from_other_dir(self):
        self.clean_auto_funcs()

        old_dir = os.getcwd()
        os.chdir("/tmp")

        transpiler.main(["--format", "--input-files", old_dir + "/tests/files/transpiler/auto_functions/wait.func"])

        assert os.path.isfile("/tmp/auto_functions.cpp")
        assert os.path.isfile("/tmp/auto_functions.h")
        assert "Wait" in open("/tmp/auto_functions.h").read()
        assert "Wait" in open("/tmp/auto_functions.cpp").read()

        os.chdir(old_dir)

    def test_transpiler_handles_whitespace(self):
        self.clean_auto_funcs()

        transpiler.main(["--input-files", "tests/files/transpiler/auto_functions/with_whitespace.func", "--output-cpp", "/tmp/auto_functions.cpp", "--output-header", "/tmp/auto_functions.h"])

        assert os.path.isfile("/tmp/auto_functions.cpp")
        assert os.path.isfile("/tmp/auto_functions.h")
        assert "Whitespace" in open("/tmp/auto_functions.h").read()
        assert "Whitespace" in open("/tmp/auto_functions.cpp").read()
        for x in range(1,6):
            assert "//PASS" + str(x) in open("/tmp/auto_functions.cpp").read()

    def test_run_transpiler_as_process(self):
        self.clean_auto_funcs()

        subprocess.check_call(["./transpiler.py", "--format", "--input-files", "tests/files/transpiler/auto_functions/wait.func", "--output-cpp", "/tmp/auto_functions.cpp", "--output-header", "/tmp/auto_functions.h"])

        assert os.path.isfile("/tmp/auto_functions.cpp")
        assert os.path.isfile("/tmp/auto_functions.h")
        assert "Wait" in open("/tmp/auto_functions.h").read()
        assert "Wait" in open("/tmp/auto_functions.cpp").read()

    def test_transpiler_silent_flag(self):
        self.clean_auto_funcs()

        assert subprocess.check_output(["./transpiler.py", "-qqq", "--output-cpp", "/tmp/auto_functions.cpp", "--output-header", "/tmp/auto_functions.h"]).decode("utf-8") == ""

    def test_transpiler_deterministic_outupt(self):
        self.clean_auto_funcs()

        os.mkdir("/tmp/auto_files")

        for n in range(1,10):
            os.mkdir("/tmp/auto_files/{}".format(n))
            transpiler.main(["--format", "--input-files", "tests/files/transpiler/auto_functions_compile/mock.func", "--output-cpp", "/tmp/auto_files/{}.cpp".format(n), "--output-header", "/tmp/auto_files/{}.h".format(n)])

        for n in range(1,9):
            assert subprocess.check_output(["diff", "/tmp/auto_files/{}".format(n), "/tmp/auto_files/{}".format(n+1)]).decode("utf-8") == ""
