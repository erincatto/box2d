rem Use this batch file to build box2d for Visual Studio
rmdir /s /q build
mkdir build
cd build
cmake ..
