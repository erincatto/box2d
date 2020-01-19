# FAQ

## What is Box2D?

Box2D is a feature rich 2D rigid body physics engine, written in C++ by Erin Catto. It has been used in many games, including Crayon Physics Deluxe, winner of the 2008 Independant Game Festival Grand Prize.

Box2D uses the [MIT license](https://en.wikipedia.org/wiki/MIT_License) license and can be used free of charge.

## What platforms does Box2D support?

Box2D is developed on Windows using Visual C++. Ports are also available for Flash, Java, C#, Python.

Erin Catto maintains the C++ version, but provides no support for other languages. Other languages are supported by the community and possibly by the authors of those ports.

## Who makes it?

Erin Catto is the driving force behind Box2D, with various others supporting the ports. Box2D is an open source project, and accepts community feedback.

## How do I get help?

You should read the documentation and the rest of this FAQ first. Also, you should study the examples included in the source distribution. Then you can visit the [subreddit](https://www.reddit.com/r/box2d/) to ask any remaining questions.

Please to not PM or email Erin Catto for support. It is best to ask questions in the forum so that everyone can benefit from the discussion.

## Documentation

### Why isn't feature foo documented?

If you grab the latest code from the git master branch you will likely find features that are not documented in the manual. New features are added to the manual after they are mature and a new point release is imminent. However, all major features added to Box2D are accompanied by example code in the testbed to test the feature and show the intended usage.

## Prerequisites

### Programming

You should have a working knowledge of C++ before you use Box2D. You should understand classes, inheritance, and pointers. There are plenty of resources on the web for learning C++. You should also understand your development environment: compilation, linking, and debugging.

### Math and Physics

You should have a basic knowledge of rigid bodies, force, torque, and impulses. If you come across a math or physics concept you don't understand, please read about it on Wikipedia. Visit this [page](http://box2d.org/publications/) if you want a deeper knowledge of the algorithms used in Box2D.

## API

### What units does Box2D use?

Box2D is tuned for meters-kilograms-seconds (MKS). Your moving objects should be between 0.1 - 10 meters. Do not use pixels as units! You will get a jittery simulation.

### How do I convert pixels to meters?

Suppose you have a sprite for a character that is 100x100 pixels. You decide to use a scaling factor that is 0.01. This will make the character physics box 1m x 1m. So go make a physics box that is 1x1. Now suppose the character starts out at pixel coordinate (345,679). So position the physics box at (3.45,6.79). Now simulate the physics world. Suppose the character physics box moves to (2.31,4.98), so move your character sprite to pixel coordinates (231,498).
Now the only tricky part is choosing a scaling factor. This really depends on your game. You should try to get your moving objects in the range 0.1 - 10 meters, with 1 meter being the sweet spot.

### Why don't you use this awesome C++ feature?

Box2D is designed to be portable, so I try to keep the C++ usage simple. Also, I don't use the STL (except sort) or other libraries to keep the dependencies low. I keep template usage low and don't use name spaces. Remember, just because a C++ feature exists, that doesn't mean you need to use it.

The many ports of Box2D to other languages platforms shows that this strategy has been successful.

### Can I use Box2D in a DLL?

Box2D was not designed to be used in a DLL. You may have to change how static data is used to make this work.

### Is Box2D thread-safe?

No. Box2D will likely never be thread-safe. Box2D has a large API and trying to make such an API thread-safe would have a large performance and complexity impact.

## Build Issues

### Why doesn't my code compile and/or link?

There are many reasons why a build can go bad. Here are a few that have come up:

* Using old Box2D headers with new code
* Not linking the Box2D library with your application
* Using old project files that don't include some new source files

## Rendering

### What are Box2D's rendering capabilities?

Box2D is only a physics engine. How you draw stuff is up to you.

### But the Testbed draws stuff

Visualization is very important for debugging collision and physics. I wrote the test bed to help me test Box2D and give you examples of how to use Box2D. The TestBed is not part of the Box2D library.

### How do I draw shapes?

Drawing shapes is not supported and shape internal data is likely to change. Instead you should implement the `b2DebugDraw` interface.

## Accuracy

Box2D uses approximate methods for a few reasons.

* Performance
* Some differential equations don't have known solutions
* Some constraints cannot be determined uniquely

What this means is that constraints are not perfectly rigid and sometimes you will see some bounce even when the restitution is zero.
Box2D uses Gauss-Seidel to approximately solve constraints.
Box2D also uses Semi-implicit Euler to approximately solve the differential equations.
Box2D also does not have exact collision. Polygons are covered with a thin skin (around 0.5cm thick) to avoid numerical problems. This can sometimes lead to unexpected contact normals. Also, some shapes may begin to overlap and then be pushed apart by the solver.

## Making Games

### Worms Clones

Making a worms clone requires arbitrarily destructible terrain. This is beyond the scope of Box2D, so you will have to figure out how to do this on your own.

### Tile Based Environment

Using many boxes for your terrain may not work well because box-like characters can get snagged on internal corners. A future update to Box2D should allow for smooth motion over edge chains. In general you should avoid using a rectangular character because collision tolerances will still lead to undesirable snagging.

### Asteroid Type Coordinate Systems

Box2D does not have any support for coordinate frame wrapping. You would likely need to customize Box2D for this purpose. You may need to use a different broad-phase for this to work.

## Determinism

### Is Box2D deterministic?

For the same input, and same binary, Box2D will reproduce any simulation. Box2D does not use any random numbers nor base any computation on random events (such as timers, etc).

However, people often want more stringent determinism. People often want to know if Box2D can produce identical results on different binaries and on different platforms. The answer is no. The reason for this answer has to do with how floating point math is implemented in many compilers and processors. I recommend reading this article if you are curious: http://www.yosefk.com/blog/consistency-how-to-defeat-the-purpose-of-ieee-floating-point.html

### But I really want determinism

This naturally leads to the question of fixed-point math. Box2D does not support fixed-point math. In the past Box2D was ported to the NDS in fixed-point and apparently it worked okay. Fixed-point math is slower and more tedious to develop, so I have chosen not to use fixed-point for the development of Box2D.

## Why is the restitution/friction mixing inaccurate?

A physically correct restitution value must be measured in experiments. But as soon as you change the geometry from the experiment then the value is wrong. Next, adding simultaneous collision makes the answer worse. We've been down this road before.

So the question of accuracy has been answered: failure.

The only remaining question is how do we make it convenient. On this opinions may vary.

`b2Settings` is just that. Settings you can adjust if you know what you are doing.

## What are the biggest mistakes made by new users?

* Using pixels for length instead of meters.
* Expecting Box2D to give pixel perfect results.
* Using b2Polygon to create concave polygons.
* Testing their code in release mode.
* Not learning C++ before using Box2D.
* Not reading this FAQ. :)
