#!/usr/bin/env bash
# Format bitrl's own code with clang-format.
# Usage: ./format.sh           format the files in place
#        ./format.sh --check   only report the files that are not formatted
#
# Use clang-format 14, as the CI does; other versions format some code
# differently. pip install clang-format==14.0.6 gives you that version, and
# CLANG_FORMAT=<path> picks the binary to use. Only files tracked by git are
# formatted.
set -euo pipefail
cd "$(dirname "$0")"

CLANG_FORMAT="${CLANG_FORMAT:-clang-format}"
version="$("$CLANG_FORMAT" --version | grep -oE '[0-9]+\.[0-9]+\.[0-9]+' | head -1)"
if [ "${version%%.*}" != "14" ]; then
    echo "warning: $CLANG_FORMAT is version $version, the CI uses 14" >&2
fi

mapfile -t files < <(git ls-files -- 'include/*.h' 'include/*.hpp' 'src/*.cpp' 'src/*.h' \
    'examples/*.cpp' 'examples/*.h' 'tests/*.cpp' 'tests/*.h')

if [ "${1:-}" = "--check" ]; then
    if ! "$CLANG_FORMAT" --dry-run -Werror "${files[@]}"; then
        echo "error: run ./format.sh to format the files above" >&2
        exit 1
    fi
    echo "All ${#files[@]} files are formatted"
else
    "$CLANG_FORMAT" -i "${files[@]}"
fi
