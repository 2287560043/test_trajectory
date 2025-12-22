#!/bin/bash

# This script is used to build the project.

# If no param passed, just build. Else look up the param and build.

if [ $# -eq 0 ]; then
    echo "Building the project..."
    colcon build --symlink-install \
        --cmake-args -DCMAKE_BUILD_TYPE=Debug \
                     -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
                     -DCMAKE_CXX_FLAGS="-Wall -Wextra -Wpedantic" \
                     -DCMAKE_C_FLAGS="-Wall -Wextra -Wpedantic" \
                     -DCMAKE_GENERATOR="Ninja" \
                     -DCMAKE_CXX_COMPILER="clang++" \
                     -DCMAKE_C_COMPILER="clang"
    exit 0
else
    if [ "$1" == "clean" ]; then
        echo "Cleaning the project..."
        rm -rf build install log
        exit 0
    elif [ "$1" == "release" ]; then
        echo "Building the project in release mode..."
        colcon build --symlink-install \
            --cmake-args -DCMAKE_BUILD_TYPE=Release \
                         -DCMAKE_EXPORT_COMPILE_COMMANDS=ON \
                         -DCMAKE_CXX_FLAGS="-Wall -Wextra -Wpedantic" \
                         -DCMAKE_C_FLAGS="-Wall -Wextra -Wpedantic" \
                         -DCMAKE_GENERATOR="Ninja" \
                         -DCMAKE_CXX_COMPILER="clang++" \
                         -DCMAKE_C_COMPILER="clang"
        exit 0
    elif [ "$1" == "--help" -o "$1" == "-h" ]; then
        echo "Usage: ./build.sh [clean|release|--help|-h]"
        exit 0
    else
        echo "Invalid argument. Use './build.sh --help' for more information."
        exit 1
    fi
fi
