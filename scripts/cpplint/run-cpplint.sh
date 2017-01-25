find . -type f | grep -v '^\./\.git' | grep -v '^\./third_party' | grep -v '^\./tools' | grep '.[cpp|h]$' | xargs ./scripts/cpplint/cpplint.py --linelength=110 --filter=-legal/copyright
