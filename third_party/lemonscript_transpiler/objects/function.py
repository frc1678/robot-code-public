from objects.file import File
from .logger import Logger

class Function(object):
    """
    Represents a .func file

    text - text passed into the constructor
    get_name() - the name of the function, as it will be used in the .auto file
    get_args() - A list of all args. Example:
                 [["Distance", "dist"], ["Angle", "angle"]]
    get_section_code() - the literal C++ code that should be in a section
    get_section() - The text in a section
    get_includes() - A list of everything that needs to be included. Strings in
                     this list will most likely be enclosed in quotes or angle
                     brackets
    """
    def __init__(self, text, script_dir):
        """
        Parses a string from a .func file into a easy to use format

        This sets self.text, then runs any functions needed to set up member
        variables by parsing self.text.
        """
        self.text = text
        self.script_dir = script_dir + ("/" if script_dir[-1] != "/" else "")

    def get_section(self, section_name):
        """
        Gets a "section" in self.text. Example of a section:

        name {
          ...
          stuff
          ...
        }

        That would be a section with the name "name".

        Sections that are used are "include", "init", and "periodic".
        """
        found_section = False
        text_in_section = []
        for line in self.text.split("\n"):
            if line != "":
                if not found_section and \
                   line.strip()[:len(section_name)] == section_name and \
                   line.strip()[-1] == "{":
                    found_section = True
                elif found_section and line[0] != "}":
                    text_in_section.append(line)
                elif found_section and line[0] == "}":
                    return '\n'.join(text_in_section)
        Logger.warn("Section {0} not found in {1}!".format(section_name, self.get_name()))
        return '' #TODO(Wesley) Better way of indicating failure

    def get_class(self):
        class_h_skel_file = open(self.script_dir + "text_includes/auto_function_class.h.skel")
        class_h_file = File(class_h_skel_file.read())

        class_h_file.replace_text("name", self.get_name())

        for var in self.get_section("global").split("\n"):
            if var != "":
                class_h_file.insert_text("vars", var + ";")

        if len(self.get_args()) == 0:
            class_cpp_skel_file = open(self.script_dir + "text_includes/auto_function_class_no_args.cpp.skel")
        else:
            class_cpp_skel_file = open(self.script_dir + "text_includes/auto_function_class.cpp.skel")
        class_cpp_file = File(class_cpp_skel_file.read())

        class_cpp_file.replace_text("name", self.get_name())
        class_cpp_file.replace_text("init_code", self.get_section_code("init"))
        class_cpp_file.replace_text("periodic_code", self.get_section_code("periodic"))

        return [class_h_file.text, class_cpp_file.text]

    def get_generator(self):
        generator_h_skel_file = open(self.script_dir + "text_includes/auto_function_generator.h.skel").read()
        generator_cpp_skel_file = open(self.script_dir + "text_includes/auto_function_generator.cpp.skel").read()

        replacements = [["replace_text", "name", self.get_name()]]

        generator_h_file = File(generator_h_skel_file, replacements)
        generator_cpp_file = File(generator_cpp_skel_file, replacements)

        return [generator_h_file.text, generator_cpp_file.text]

    def get_section_code(self, section):
        """
        Gets the transpiled C++ code inside a section, including auto-generated
        variable casting code.
        """
        raw_code = self.get_section(section)
        var_init_lines = []

        argnum = 0

        if len(self.get_args()) > 0:
            for arg in self.get_args():
                var_cast_func = "ConvertArgs::Convert<{type}>()(ls_arg_list[{argnum}+1])" \
                        .format(type = arg[0], argnum = argnum)
                var_init_line = "  {arg[0]} {arg[1]} = {cast};".format(arg=arg, cast=var_cast_func)
                var_init_lines.append(var_init_line)
                argnum += 1

        return ("  // BEGIN AUTO GENERATED CODE\n" +
                "\n".join(var_init_lines) +
                "\n  // END AUTO GENERATED CODE\n\n" +
                raw_code)

    def get_includes(self):
        """
        Returns a list of all of the lines in the "include" section of self.text,
        stripping trailing commas if needed.
        """
        includes = []
        for line in self.get_section("include").split("\n"):
            if line:
                includes += [line.strip()]
        return includes

    def get_args(self):
        """
        Returns a list of arguments. For an example of the format, see the
        class docstring.
        """
        try:
            arg_string = self.get_raw_constructor().split("(")[1].split(")")[0]
        except IndexError:
            arg_string = ""
        args = []
        if arg_string.strip() != "":
            for arg in arg_string.split(","):
                arg = arg.strip()
                arg_pair = [arg.strip().split(" ")[0], arg.strip().split(" ")[-1]]
                args.append(arg_pair)

        # args.insert(0, ["CitrusRobot *", "robot"])
        return args

    def get_raw_constructor(self):
        """
        Returns the first line of self.text, which we assume to be the
        lemonscript-style constructor. This is not valid C++ code.
        """
        return self.text.split("\n")[0]

    def get_name(self):
        """
        Returns the name of the function, as it will be used in lemonscript.
        """
        return self.get_raw_constructor().split("(")[0].strip()
