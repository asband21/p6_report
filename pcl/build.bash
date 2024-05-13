#!/bin/bash

# Define the build directory
BUILD_DIR="build"

# Ensure the build directory exists
if [ ! -d "$BUILD_DIR" ]; then
  mkdir "$BUILD_DIR"
fi

# Navigate to the build directory
cd "$BUILD_DIR"

# Run CMake to configure the project
cmake ..

# Clean previous builds
make clean

# Build the project
make

# Navigate back to the root directory
cd ..

# Ensure the program is run with the correct arguments
if [ "$#" -lt 2 ]; then
  echo "Usage: $0 <path_to_csv> <test_runs>"
  exit 1
fi

# Run the program with the provided arguments
./build/ICPExample "$1" "$2"
