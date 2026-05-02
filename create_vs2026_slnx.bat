rem Use this batch file to build box2d for Visual Studio 2026
rmdir /s /q build
cmake -S . -B build -G "Visual Studio 18 2026"
