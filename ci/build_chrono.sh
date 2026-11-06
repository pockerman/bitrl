#!/usr/bin/env bash
# Build Project Chrono and install it under the given prefix.
# Usage: ci/build_chrono.sh <install-prefix>
#
# The commit is pinned rather than a release tag because bitrl uses the
# imported targets and the API of Chrono's main branch. The 9.0.1 tag
# does not provide them even though this commit also reports version 9.0.1.
# The CI cache key depends on this file, so changing the commit rebuilds Chrono.
set -euo pipefail

CHRONO_COMMIT="13b9c9f345ee3e1dae4169eb1978dc86050e69a3"

prefix="${1:?usage: build_chrono.sh <install-prefix>}"
work_dir="$(mktemp -d)"
trap 'rm -rf "$work_dir"' EXIT

git init --quiet "$work_dir/chrono"
git -C "$work_dir/chrono" remote add origin https://github.com/projectchrono/chrono.git
git -C "$work_dir/chrono" fetch --quiet --depth 1 origin "$CHRONO_COMMIT"
git -C "$work_dir/chrono" checkout --quiet FETCH_HEAD

cmake -S "$work_dir/chrono" -B "$work_dir/build" \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_INSTALL_PREFIX="$prefix" \
    -DBUILD_DEMOS=OFF \
    -DBUILD_BENCHMARKING=OFF \
    -DBUILD_TESTING=OFF \
    -DCH_ENABLE_MODULE_IRRLICHT=ON

cmake --build "$work_dir/build" --parallel "$(nproc)"
cmake --install "$work_dir/build"
