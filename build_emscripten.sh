#!/usr/bin/env bash
set -e

# Run after activating emsdk: source /path/to/emsdk/emsdk_env.sh
rm -rf build
emcmake cmake -S . -B build -DBOX2D_VALIDATE=OFF -DBOX2D_UNIT_TESTS=ON -DBOX2D_SAMPLES=OFF -DCMAKE_BUILD_TYPE=Debug
cmake --build build
