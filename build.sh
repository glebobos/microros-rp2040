#!/bin/bash
set -e

echo "Starting standalone build process for micro-ROS Pico..."

# Navigate to the mounted module directory
cd /project/src

# Clean old build if it exists
rm -rf build
mkdir build
cd build

echo "Running CMake..."
cmake .. -DPICO_SDK_PATH=/opt/pico-sdk

echo "Building firmware..."
make -j$(nproc)

echo "Build complete!"
echo "The resulting uf2 file has been stored in your src/build/ folder."
