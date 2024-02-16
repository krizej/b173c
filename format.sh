find src/ -type f -iname "*.c" -o -iname "*.h" -print0 | xargs -0 clang-format -i
