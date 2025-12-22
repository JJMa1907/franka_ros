#!/bin/bash
source /opt/ros/noetic/setup.bash
set -e

# Get the absolute path to the directory containing this script
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROTOBUF_VERSION="3.6.1"
PROTOBUF_INSTALL_DIR="${SCRIPT_DIR}/protobuf_fix"
PROTOBUF_SRC_DIR="${SCRIPT_DIR}/protobuf-${PROTOBUF_VERSION}"

echo "Protobuf version: ${PROTOBUF_VERSION}"
echo "Protobuf install directory: ${PROTOBUF_INSTALL_DIR}"
echo "Protobuf source directory: ${PROTOBUF_SRC_DIR}"

# Create install directory if it doesn't exist
mkdir -p "${PROTOBUF_INSTALL_DIR}"

# Download and extract protobuf if it doesn't exist
if [ ! -d "${PROTOBUF_SRC_DIR}" ]; then
  echo "Downloading Protobuf ${PROTOBUF_VERSION}..."
  export http_proxy="http://172.16.0.10:7890"
  export https_proxy="http://172.16.0.10:7890"
  wget "https://github.com/protocolbuffers/protobuf/archive/v${PROTOBUF_VERSION}.tar.gz" -O "protobuf-v${PROTOBUF_VERSION}.tar.gz"
  unset http_proxy
  unset https_proxy
  tar -xzvf "protobuf-v${PROTOBUF_VERSION}.tar.gz"
  rm "protobuf-v${PROTOBUF_VERSION}.tar.gz"
fi

# Build and install protobuf
echo "Building and installing Protobuf..."
cd "${PROTOBUF_SRC_DIR}/cmake"
cmake . -DCMAKE_INSTALL_PREFIX="${PROTOBUF_INSTALL_DIR}" -Dprotobuf_BUILD_TESTS=OFF
make -j$(nproc)
make install
cd "${SCRIPT_DIR}"

# Source the environment and build the catkin workspace
echo "Building the catkin workspace..."
source /opt/ros/noetic/setup.bash

# Set the CMAKE_PREFIX_PATH to include both ROS and our custom protobuf
export CMAKE_PREFIX_PATH="${PROTOBUF_INSTALL_DIR}:${CMAKE_PREFIX_PATH}"

# Clean the workspace completely by removing build and devel directories
echo "Cleaning workspace..."
rm -rf "${SCRIPT_DIR}/build" "${SCRIPT_DIR}/devel"

# Build the workspace
catkin_make -DCMAKE_BUILD_TYPE=Release \
            -DFranka_DIR:PATH=/home/william/franka/libfranka

echo "Build finished."
