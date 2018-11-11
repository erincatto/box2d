![Box2D Logo](http://box2d.org/images/icon.gif)

# Box2D 

**Box2D is a 2D physics engine for games.**

For help with Box2D, please visit http://www.box2d.org. There is a forum there where you may post your questions.

Please see [Building.md](https://github.com/erincatto/Box2D/blob/master/Building.md) to learn how to build Box2D and run the testbed.

## Demos

To run the demos, set `Testbed` as your startup project and press <kbd>F5</kbd>. Some test bed commands are:

- <kbd>r</kbd> to reset the current test
- <kbd>SPACE</kbd> to launch a bomb
- <kbd>&larr;</kbd> <kbd>&rarr;</kbd> keys to pan
- <kbd>x</kbd> and <kbd>z</kbd> to zoom in/out
- use the mouse to click and drag objects

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

### Testbed
- OpenGL with GLFW
- Graphical user interface with imgui
- Easily switch between tests using GUI
- Test framework for easily adding new tests
- Mouse picking and the bomb!
- CMake build system files

## Documentation
You can find documentation related to the project in the [documentation page](http://box2d.org/documentation/) and in the [documentation folder](https://github.com/erincatto/Box2D/tree/master/Box2D/Documentation) in GitHub
- [User manual](http://box2d.org/manual.pdf)
- [Doxygen document](https://github.com/erincatto/Box2D/blob/master/Box2D/Documentation/Doxyfile) with code comments
- [subreddit](https://www.reddit.com/r/box2d/)
- [Discord server](https://discord.gg/NKYgCBP)
- [User forum (legacy)](http://box2d.org/forum/)

You can also visit the project wiki where you will find the [FAQ](https://github.com/erincatto/Box2D/wiki/FAQ)'s page

## License

Box2D is developed by Erin Catto, and has the [zlib license](http://en.wikipedia.org/wiki/Zlib_License). While the zlib license does not require acknowledgement, we encourage you to give credit to Box2D in your product.
