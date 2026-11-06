#!/usr/bin/env bash
# Install the bitrl build dependencies on Ubuntu.
# Usage: ci/install_deps.sh [--with-chrono-deps]
set -euo pipefail

# Keep these in sync with what the code is developed against.
PAHO_C_TAG="v1.3.15"
PAHO_CPP_TAG="v1.5.3"

SUDO=""
if [ "$(id -u)" -ne 0 ]; then
    SUDO="sudo"
fi

packages=(build-essential cmake git libssl-dev libboost-all-dev libgtest-dev
          libeigen3-dev libblas-dev libopencv-dev)

if [ "${1:-}" = "--with-chrono-deps" ]; then
    packages+=(libirrlicht-dev libgl1-mesa-dev libglu1-mesa-dev libxxf86vm-dev)
fi

$SUDO apt-get update
$SUDO env DEBIAN_FRONTEND=noninteractive apt-get install -y "${packages[@]}"

if [ "${1:-}" = "--with-chrono-deps" ]; then
    # irrXML.h declares its constructors as xmlChar<T>() which is not valid in
    # C++20 and bitrl is compiled as C++20. This is idempotent.
    $SUDO sed -i 's/xmlChar<T>(/xmlChar(/' /usr/include/irrlicht/irrXML.h
fi

work_dir="$(mktemp -d)"
trap 'rm -rf "$work_dir"' EXIT

install_paho() {
    local repo="$1" tag="$2"
    git clone --quiet --depth 1 --branch "$tag" "https://github.com/eclipse/${repo}.git" "$work_dir/$repo"
    cmake -S "$work_dir/$repo" -B "$work_dir/$repo/build" \
        -DCMAKE_BUILD_TYPE=Release \
        -DPAHO_WITH_SSL=ON \
        -DPAHO_BUILD_SHARED=ON \
        -DPAHO_BUILD_STATIC=OFF
    cmake --build "$work_dir/$repo/build" --parallel "$(nproc)"
    $SUDO cmake --install "$work_dir/$repo/build"
}

install_paho paho.mqtt.c "$PAHO_C_TAG"
install_paho paho.mqtt.cpp "$PAHO_CPP_TAG"
$SUDO ldconfig
