#!/usr/bin/env bash
# Install bitrl from a build directory into a temporary prefix, then build and
# run tests/install_consumer against it the way another project would.
# Usage: ci/check_install.sh <build-dir> [prefixes of the dependencies, ;-separated]
#        for example ci/check_install.sh build "$HOME/opt/bitsim;$HOME/opt/libtorch"
set -euo pipefail

build_dir="$(realpath "${1:?usage: check_install.sh <build-dir> [dependency-prefixes]}")"
dependency_prefixes="${2:-}"
root="$(cd "$(dirname "$0")/.." && pwd)"

work_dir="$(mktemp -d)"
trap 'rm -rf "$work_dir"' EXIT

cmake --install "$build_dir" --prefix "$work_dir/prefix" > /dev/null
cmake -S "$root/tests/install_consumer" -B "$work_dir/build" \
    -DCMAKE_PREFIX_PATH="$work_dir/prefix${dependency_prefixes:+;$dependency_prefixes}"
cmake --build "$work_dir/build"
"$work_dir/build/install_consumer"
echo "The installed bitrl works"
