#!/usr/bin/env bash

# Use this to build box2d on any system with a bash shell
rm -rf build
mkdir build
cd build

# I haven't been able to get Wayland working on WSL but X11 works.
# https://www.glfw.org/docs/latest/compile.html
cmake -DBOX2D_BUILD_DOCS=OFF -DGLFW_BUILD_WAYLAND=OFF ..
cmake --build .
