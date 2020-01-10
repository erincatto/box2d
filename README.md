![Box2D Logo](http://box2d.org/images/icon.gif)

# Build Status
[![Build Status](https://travis-ci.org/erincatto/box2d.svg?branch=master)](https://travis-ci.org/erincatto/box2d)

# Box2D 

**Box2D is a 2D physics engine for games.**

## Contributing

Please do not submit pull requests with new features. Instead, please file an issue first for discussion. For bugs, I prefer detailed bug reports over pull requests.

## Features

### Collision
- Continuous collision detection
- Contact callbacks: begin, end, pre-solve, post-solve
- Convex polygons and circles
- Multiple shapes per body
- One-shot contact manifolds
- Dynamic tree broadphase
- Efficient pair management
- Fast broadphase AABB queries
- Collision groups and categories

### Physics
- Continuous physics with time of impact solver
- Persistent body-joint-contact graph
- Island solution and sleep management
- Contact, friction, and restitution
- Stable stacking with a linear-time solver
- Revolute, prismatic, distance, pulley, gear, mouse joint, and other joint types
- Joint limits, motors, and friction
- Momentum decoupled position correction
- Fairly accurate reaction forces/impulses

### System
- Small block and stack allocators
- Centralized tuning parameters
- Highly portable C++ with no use of STL containers

### Samples
- OpenGL with GLFW
- Graphical user interface with imgui
- Extensible test framework
- Support for loading world dumps

## Building
- Install [CMake](https://cmake.org/)
- Ensure CMake is in the user `PATH`
- Visual Studio: run `build.bat` from the command prompt
- Otherwise: run `build.sh` from a bash shell
- Results are in the build sub-folder
- On Windows you can open box2d.sln

## Documentation
- [User manual](https://github.com/erincatto/box2d/tree/master/docs)
- [subreddit](https://www.reddit.com/r/box2d/)
- [Discord server](https://discord.gg/NKYgCBP)

You can also visit the project wiki where you will find the [FAQ](https://github.com/erincatto/Box2D/wiki/FAQ)'s page

## License

Box2D is developed by Erin Catto, and uses the [MIT license](https://en.wikipedia.org/wiki/MIT_License).