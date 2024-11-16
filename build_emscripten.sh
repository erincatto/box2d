#!/usr/bin/env bash

# source emsdk_env.sh

# Use this to build box2d on any system with a bash shell
rm -rf build
mkdir build
cd build
emcmake cmake -DBOX2D_VALIDATE=OFF -DBOX2D_UNIT_TESTS=ON -DBOX2D_SAMPLES=OFF -DCMAKE_BUILD_TYPE=Debug ..
cmake --build .
