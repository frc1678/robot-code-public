import sys
import os

parent_dir = os.path.dirname(os.path.realpath(__file__)) + "/../"
sys.path.append(parent_dir) # a bit of a hack, but it makes the import the same
from objects.file import File

def get_file_path(path):
    return os.path.dirname(os.path.realpath(__file__)) + "/" + path


class TestFileMethods:

    def test_init(self):
        test_file = open(get_file_path("files/file/init.skel"))
        test_text = test_file.read()

        file_object = File(test_text)

        assert file_object.text == test_text
        assert file_object.init_text == test_text

    def test_init_replacements(self):
        test_file = open(get_file_path("files/file/init.skel"))
        test_text = test_file.read()

        replacements = [
            ["replace_text", "key", "value"],
            ["replace_file", "key2", get_file_path("files/file/init.inc")]
        ]
        file_object = File(test_text, replacements)

        result = test_text.replace("<<<key>>>", "value").replace("<<<key2>>>", open(get_file_path("files/file/init.inc")).read())

        assert file_object.text == result

    def test_insert_text(self):
        test_file = open(get_file_path("files/file/insert.skel"))
        test_text = test_file.read()

        file_object = File(test_text)
        file_object.insert_text("insert", "test")

        result = open(get_file_path("files/file/insert_result.skel")).read()

        assert file_object.text == result

    def test_insert_file(self):
        test_file = open(get_file_path("files/file/insert.skel"))
        test_text = test_file.read()

        file_object = File(test_text)
        file_object.insert_file("insert", get_file_path("files/file/insert.inc"))

        result = open(get_file_path("files/file/insert_file_result.skel")).read()

        assert file_object.text == result

    def test_remove_modelines(self):
        test_file = open(get_file_path("files/file/modelines.skel"))
        test_text = test_file.read()

        result = "\n".join(test_text.split("\n")[5:])

        file_object = File(test_text, remove_modelines=0)

        file_object.remove_modelines()

        assert file_object.text == result

    def test_remove_modelines_custom_number(self):
        test_file = open(get_file_path("files/file/modelines.skel"))
        test_text = test_file.read()

        result = ""

        file_object = File(test_text, remove_modelines=0)

        file_object.remove_modelines(n=10)

        assert file_object.text == result

    def test_remove_modelines_by_default(self):
        test_file = open(get_file_path("files/file/modelines.skel"))
        test_text = test_file.read()

        result = "\n".join(test_text.split("\n")[5:])

        file_object = File(test_text)

        assert file_object.text == result
