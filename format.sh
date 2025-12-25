find src \
  -type f \( -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) \
  ! -path "*/extern/*" \
  -exec clang-format -i {} +

