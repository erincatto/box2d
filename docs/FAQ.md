# FAQ

## What is Box2D?
Box2D is a feature rich 2D rigid body physics engine, written in C11 by Erin Catto. It has been used in many games and in many
game engines.

Box2D uses the [MIT license](https://en.wikipedia.org/wiki/MIT_License) license and can be used free of charge. Credit
should be included if possible. Support is [appreciated](https://github.com/sponsors/erincatto). You may use the
Box2D [logo](https://box2d.org/images/logo.svg).

## What platforms does Box2D support?
Box2D is developed using C11. Ports and bindings are likely available for most languages and platforms.

Erin Catto maintains the C11 version, but provides no support for other languages. Other languages are supported
by the community and possibly by the authors of those ports.

## Who makes it?
Erin Catto is the creator and sole contributor of the C11 version of Box2D, with various others supporting the ports. Box2D is an open source project, and accepts community feedback.

## How do I get help?
You should read the documentation and the rest of this FAQ first. Also, you should study the examples included in the source distribution. Then you can visit the [Discord](https://discord.gg/aM4mRKxW) to ask any remaining questions.

Please to not message or email Erin Catto for support. It is best to ask questions on the Discord server so that everyone can benefit from the discussion.

## Documentation

### Why isn't a feature documented?
If you grab the latest code from the git master branch you will likely find features that are not documented in the manual. New features are added to the manual after they are mature and a new point release is imminent. However, all major features added to Box2D are accompanied by example code in the samples application to test the feature and show the intended usage.

## Prerequisites

### Programming
You should have a working knowledge of C before you use Box2D. You should understand functions, structures, and pointers. There are plenty of resources on the web for learning C. You should also understand your development environment: compilation, linking, and debugging.

### Math and Physics
You should have a basic knowledge of rigid bodies, force, torque, and impulses. If you come across a math or physics concept you don't understand, please read about it on Wikipedia. Visit this [page](http://box2d.org/publications/) if you want a deeper knowledge of the algorithms used in Box2D.

## API

### What units does Box2D use?
Box2D is tuned for meters-kilograms-seconds (MKS). This is recommend as the units for your game. However, you may use
different units if you are careful.

### How do I convert pixels to meters?
Suppose you have a sprite for a character that is 100x100 pixels. You decide to use a scaling factor that is 0.01. This will make the character physics box 1m x 1m. So go make a physics box that is 1x1. Now suppose the character starts out at pixel coordinate (345,679). So position the physics box at (3.45,6.79). Now simulate the physics world. Suppose the character physics box moves to (2.31,4.98), so move your character sprite to pixel coordinates (231,498).

Now the only tricky part is choosing a scaling factor. This really depends on your game. You should try to get your moving objects in the range 0.1 - 10 meters, with 1 meter being the sweet spot.

This [repo](https://github.com/erincatto/box2d-raylib) shows how to convert meters to pixels.

### Why don't you use this awesome language?
Box2D is designed to be portable and easy to wrap with other languages, so I decided to use C11. I used C11 to get support for atomics.

### Can I use Box2D in a DLL?
Yes. See the CMake option `BUILD_SHARED_LIBS`.

### Is Box2D thread-safe?
No. Box2D will likely never be thread-safe. Box2D has a large API and trying to make such an API thread-safe would have a large performance and complexity impact. However, you can call read only functions from multiple threads. For example, all the
[spatial query](#spatial) functions are read only.

## Build Issues

### Why doesn't my code compile and/or link?
There are many reasons why a build can go bad. Here are a few that have come up:
* Using old Box2D headers with new code
* Not linking the Box2D library with your application
* Using old project files that don't include some new source files

## Rendering

### What are Box2D's rendering capabilities?
Box2D is only a physics engine. How you draw stuff is up to you.

### But the samples application draws stuff
Visualization is very important for debugging collision and physics. I wrote the samples application to help me test Box2D and give you examples of how to use Box2D. The samples are not part of the Box2D library.

### How do I draw shapes?
Implement the `b2DebugDraw` interface and call `b2World_Draw()`.

## Accuracy
Box2D uses approximate methods for a few reasons.
* Performance
* Some differential equations don't have known solutions
* Some constraints cannot be determined uniquely

What this means is that constraints are not perfectly rigid and sometimes you will see some bounce even when the restitution is zero.
Box2D uses [Gauss-Seidel](https://en.wikipedia.org/wiki/Gauss%E2%80%93Seidel_method) to approximately solve constraints.
Box2D also uses [Semi-implicit Euler](https://en.wikipedia.org/wiki/Semi-implicit_Euler_method) to approximately solve the differential equations.
Box2D also does not have exact collision. There is no continuous collision between dynamic shapes. Slow moving shapes may have small overlap for a few time steps. In extreme stacking scenarios, shapes may have sustained overlap.

## Making Games

### Worms Clones
Making a worms clone requires arbitrarily destructible terrain. This is beyond the scope of Box2D, so you will have to figure out how to do this on your own.

### Tile Based Environment
Using many boxes for your terrain may not work well because box-like characters can get snagged on internal corners. Box2D proves chain shapes for smooth collision, see `b2ChainDef`. In general you should avoid using a rectangular character because collision tolerances will still lead to undesirable snagging. Box2D provides capsules and rounded polygons that may work better for characters.

### Asteroid Type Coordinate Systems
Box2D does not have any support for coordinate frame wrapping. You would likely need to customize Box2D for this purpose. You may need to use a different broad-phase for this to work.

## Determinism

### Is Box2D deterministic?
For the same input, and same binary, Box2D will reproduce any simulation. Box2D does not use any random numbers nor base any computation on random events (such as timers, etc).

Box2D is also deterministic under multithreading. A simulation using two threads will give the same result as eight threads.

However, people often want more stringent determinism. People often want to know if Box2D can produce identical results on different binaries and on different platforms. Currently this is not provided, but the situation may improve in a future update.

### But I really want determinism
This naturally leads to the question of fixed-point math. Box2D does not support fixed-point math. In the past Box2D was ported to the NDS in fixed-point and apparently it worked okay. Fixed-point math is slower and more tedious to develop, so I have chosen not to use fixed-point for the development of Box2D.

## What are the common mistakes made by new users?
* Using pixels for length instead of meters
* Expecting Box2D to give pixel perfect results
* Testing their code in release mode
* Not learning C before using Box2D
