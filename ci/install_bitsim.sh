#!/usr/bin/env bash
# Build bitsim, the robotics library (CMake package bitsim), without Chrono
# and install it into <prefix>. Its own dependencies must already be
# installed, see bitsim's ci/install_deps.sh.
# Usage: ci/install_bitsim.sh <prefix> [git ref, default master]
# Then configure bitrl with -DCMAKE_PREFIX_PATH=<prefix>
set -euo pipefail

prefix="${1:?usage: install_bitsim.sh <prefix> [git ref]}"
ref="${2:-master}"

work_dir="$(mktemp -d)"
trap 'rm -rf "$work_dir"' EXIT

git clone --quiet --depth 1 --branch "$ref" https://github.com/pockerman/bitsim.git "$work_dir/bitsim"
cmake -S "$work_dir/bitsim" -B "$work_dir/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DENABLE_CHRONO=OFF \
    -DENABLE_EXAMPLES=OFF \
    -DENABLE_TESTS=OFF
cmake --build "$work_dir/build" --parallel "$(nproc)"
cmake --install "$work_dir/build" --prefix "$prefix"
echo "bitsim ($ref) is installed in $prefix"
