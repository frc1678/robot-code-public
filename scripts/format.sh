if [ $1 == "--help" -o $1 == "-h" -o $1 == "-help" ]
then
echo "This script is a wrapper around clang-format that will format the code with the Google C++ style guide, regardless of what version of clang-format is installed. It takes no arguments: simply run it, and it will format all of the code in the current directory."
exit 0
fi

if [ -f /usr/bin/clang-format-3.6 ]
then
  export CLANG_PATH="/usr/bin/clang-format-3.6"
fi

if [ -f /usr/bin/clang-format-3.7 ]
then
  export CLANG_PATH="/usr/bin/clang-format-3.7"
fi

if [ -f /usr/bin/clang-format ]
then
  export CLANG_PATH="/usr/bin/clang-format"
fi

if [ -f /usr/local/bin/clang-format ]
then
  export CLANG_PATH="/usr/local/bin/clang-format"
fi

$CLANG_PATH -i $(find . | grep "\(\.cpp\|\.h\)\$") -style=Google
