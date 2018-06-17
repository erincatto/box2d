# Building Box2D

The Box2D library is easy to build. I recommend adding the source files in the Box2D folder into your build system or project files for your IDE. The Box2D library is portable, so you should not need to configure it for your platform.

Building the testbed is optional and may not work on your platform. I use [premake5](https://premake.github.io/) to generate projects for the testbed. You can get premake5 here: https://premake.github.io/. The testbed requires OpenGL 3.3.

I support premake working well for [Visual Studio](https://www.visualstudio.com) on Windows 10 and [Xcode](https://developer.apple.com/xcode/) on MacOS. I appreciate contributions to get premake working on other platforms.

### Visual Studio
Here are the steps for Visual Studio 2017:
- Command line: `premake5 vs2017`
- Open the resulting `Box2D.sln` (should be in `Build/vs2017`)
- Set Testbed as the startup project
- In the Testbed debugging properties, set the Working Directory to `..\Testbed`
- Press <kbd>F5</kbd> to build and run the Testbed

### Xcode
Here are the steps for Xcode:
- Command line: `premake5 xcode4`
- Open the resulting project file (should be `Build/xcode4`)
- Set the Testbed as the current Scheme
- Edit the Testbed Scheme, in the Run Options, use a custom working directory
- Set the Testbed directory as the working directory
- Press <kbd>Command</kbd>-<kbd>R</kbd> to build and run the Testbed

### Linux
Here are the steps for Linux:
- Command line: `premake gmake`
- Command line: `make -C Build`
- Command line: `cd Testbed`
- Command line: `../Build/bin/x86_64/Debug/Testbed`

If using Mesa, you may need to override the OpenGL version.
- Command line: `MESA_GL_VERSION_OVERRIDE=3.3COMPAT ../Build/bin/x86_64/Debug/Testbed`

Thanks,
Erin
