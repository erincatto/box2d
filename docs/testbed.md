# Testbed
Once you have conquered the HelloWorld example, you should start looking
at Box2D's testbed. The testbed is a testing framework and demo
environment. Here are some of the features:
- Camera with pan and zoom.
- Mouse picking of shapes attached to dynamic bodies.
- Extensible set of tests.
- GUI for selecting tests, parameter tuning, and debug drawing options.
- Pause and single step simulation.
- Text rendering.

![Box2D Testbed](images/testbed.gif)

The testbed has many examples of Box2D usage in the test cases and the
framework itself. I encourage you to explore and tinker with the testbed
as you learn Box2D.

Note: the testbed is written using [GLFW](https://www.glfw.org) and
[imgui](https://github.com/ocornut/imgui). The testbed is not part of the
Box2D library. The Box2D library is agnostic about rendering. As shown by
the HelloWorld example, you don't need a renderer to use Box2D.
