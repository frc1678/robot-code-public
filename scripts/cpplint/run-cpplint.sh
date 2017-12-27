# Get all C++ files not in ./.git, ./third_party, or ./tools
find . -type f | grep -Ev '^\./(\.git|third_party|tools)' | grep -E '\.(h|hpp|cpp|cc)$' |
# Send them to cpplint
xargs ./scripts/cpplint/cpplint.py --linelength=110 --filter=-legal/copyright,-build/c++11 --extension=h,hpp,cpp,cc |
# Remove "Done processing file" and check that there are 0 errors.
# Problems are printed anyway since they are sent to stderr.
tail -n 1 | grep "found: 0"
