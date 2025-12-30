find src \
  -type f \( -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) \
  ! -path "*/extern/*" \
  -exec clang-format -i {} +

find examples \
  -type f \( -name "*.cpp" -o -name "*.h" -o -name "*.hpp" \) \
  ! -path "*/webots/*" \
  -exec clang-format -i {} +

