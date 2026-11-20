#!/usr/bin/env bash
# Download the CPU build of libtorch into <dir>.
# Usage: ci/install_libtorch.sh <dir>
# Then configure cuberl with -DCMAKE_PREFIX_PATH=<dir>/libtorch
set -euo pipefail

# Keep this in sync with what the code is developed against
LIBTORCH_VERSION="2.5.1"

dir="${1:?usage: install_libtorch.sh <dir>}"
mkdir -p "$dir"

url="https://download.pytorch.org/libtorch/cpu/libtorch-cxx11-abi-shared-with-deps-${LIBTORCH_VERSION}%2Bcpu.zip"
curl --fail --silent --show-error --location --output "$dir/libtorch.zip" "$url"
unzip -q "$dir/libtorch.zip" -d "$dir"
rm "$dir/libtorch.zip"
echo "libtorch ${LIBTORCH_VERSION} is in $dir/libtorch"
