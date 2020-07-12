#!/usr/bin/env bash

# Builds Box2D along with documentation
rm -rf build
mkdir build
cd build
cmake -DBOX2D_BUILD_DOCS=ON ..
cmake --build .
