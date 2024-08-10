# Overview
Box2D is a 2D rigid body simulation library for games. Programmers can
use it in their games to make objects move in realistic ways and make
the game world more interactive. From the game engine's point of view,
a physics engine is just a system for procedural animation.

Box2D is written in portable C11. Most of the types defined in the
engine begin with the b2 prefix. Hopefully this is sufficient to avoid
name clashing with your application.

## Prerequisites
In this manual I'll assume you are familiar with basic physics
concepts, such as mass, force, torque, and impulses. If not, please
first consult Google search and Wikipedia.

Box2D was created as part of a physics tutorial at the Game Developer
Conference. You can get these tutorials from the publications section of
[box2d.org](https://box2d.org/publications/).

Since Box2D is written in C, you are expected to be experienced in C
programming. Box2D should not be your first C programming project! You
should be comfortable with compiling, linking, and debugging.

> **Caution**:
> Box2D should not be your first C project. Please learn C
> programming, compiling, linking, and debugging before working with
> Box2D. There are many resources for this online.

## Scope
This manual covers the majority of the Box2D API. However, not every
aspect is covered. Please look at the Reference section and samples
application included with Box2D to learn more.

This manual is only updated with new releases. The latest version of
Box2D may be out of sync with this manual.

## Feedback and Bugs
Please file bugs and feature requests here:
[Box2D Issues](https://github.com/erincatto/box2d/issues)

You can help to ensure your issue gets fixed if you provide sufficient
detail. A testbed example that reproduces the problem is ideal. You can
read about the testbed later in this document.

There is also a [Discord server](https://discord.gg/NKYgCBP) and a
[subreddit](https://reddit.com/r/box2d) for Box2D.

## Core Concepts
Box2D works with several fundamental concepts and objects. I briefly
define these objects here and more details are given later in this
document.

### rigid body
A chunk of matter that is so strong that the distance between any two
bits of matter on the chunk is constant. They are hard like a diamond.
In the following discussion I use *body* interchangeably with rigid body.

### shape
A shape binds collision geometry to a body and adds material properties such as
density, friction, and restitution. A shape puts collision geometry into the
collision system (broad-phase) so that it can collide with other shapes.

### constraint
A constraint is a physical connection that removes degrees of freedom
from bodies. A 2D body has 3 degrees of freedom (two translation
coordinates and one rotation coordinate). If I take a body and pin it
to the wall (like a pendulum) I have constrained the body to the wall.
At this point the body can only rotate about the pin, so the constraint
has removed 2 degrees of freedom.

### contact constraint
A special constraint designed to prevent penetration of rigid bodies and
to simulate friction and restitution. You do not create contact
constraints; they are created automatically by Box2D.

### joint constraint
This is a constraint used to hold two or more bodies together. Box2D
supports several joint types: revolute, prismatic, distance, and more.
Joints may have limits, motors, and/or springs.

### joint limit
A joint limit restricts the range of motion of a joint. For example, the
human elbow only allows a certain range of angles.

### joint motor
A joint motor drives the motion of the connected bodies according to the
joint's degrees of freedom. For example, you can use a motor to drive
the rotation of an elbow. Motors have a target speed and a maximum force
or torque. The simulation will apply the force or torque required to
achieve the desired speed.

### joint spring
A joint spring has a stiffness and damping. In Box2D spring stiffness is
expressed in terms or Hertz or cycles per second. This lets you configure how
quickly a spring reacts regardless of the body masses. Joint springs also
have a damping ratio to let you specify how quickly the spring will come to
rest.

### world
A physics world is a collection of bodies, shapes, joints, and contacts
that interact together. Box2D supports the creation of multiple worlds which
are completely independent.

### solver
The physics world has a solver that is used to advance time and to
resolve contact and joint constraints. The Box2D solver is a high
performance sequential solver that operates in order N time, where N is
the number of constraints.

### continuous collision
The solver advances bodies in time using discrete time steps. Without
intervention this can lead to tunneling.
![Tunneling Effect](images/tunneling1.svg)

Box2D contains specialized algorithms to deal with tunneling. First, the
collision algorithms can interpolate the motion of two bodies to find
the first time of impact (TOI). Second, speculative collision is used to create
contact constraints between bodies before they touch.

### events
World simulation leads to the creation of events that are available at the end
of the time step:

- body movement events
- contact begin and end events
- contact hit events

These events allow your application to react to changes in the simulation.

## Modules
Box2D's primary purpose is to provide rigid body simulation. However,
there are math and collision features that may be useful apart from the
rigid body simulation. These are provided in the `include` directory. Anything
in the `include` directory is considered public, while everything in the `src`
directory is consider internal.

Public features are supported and you can get help with these on the Discord
server. Using internal code directly is not supported.

## Units
Box2D works with floating point numbers and tolerances have to be used
to make Box2D perform well. These tolerances have been tuned to work
well with meters-kilogram-second (MKS) units. In particular, Box2D has
been tuned to work well with moving shapes between 0.1 and 10 meters. So
this means objects between soup cans and buses in size should work well.
Static shapes may be up to 50 meters long without trouble. If you have a
large world, you should split it up into multiple static bodies.

Being a 2D physics engine, it is tempting to use pixels as your units.
Unfortunately this will lead to a poor simulation and possibly weird
behavior. An object of length 200 pixels would be seen by Box2D as the
size of a 45 story building.

> **Caution**: 
> Box2D is tuned for MKS units. Keep the size of moving objects larger than 1cm.
> You'll need to use some scaling system when
> you render your environment and actors. The Box2D samples application
> does this by using an OpenGL viewport transform. Do not use pixel units
> unless you understand the implications.

It is best to think of Box2D bodies as moving billboards upon which you
attach your artwork. The billboard may move in a unit system of meters,
but you can convert that to pixel coordinates with a simple scaling
factor. You can then use those pixel coordinates to place your sprites,
etc. You can also account for flipped coordinate axes.

Another limitation to consider is overall world size. If your world units
become larger than 12 kilometers or so, then the lost precision can affect
stability.

> **Caution**: 
> Box2D works best with world sizes less than 12 kilometers. If you are
> careful with your simulation tuning, this can be pushed up to around 24
> kilometers, which is much larger than most game worlds.

Box2D uses radians for angles. The body rotation is stored a complex number,
so when you access the angle of a body, it will be between \f$-\pi\f$ and \f$\pi\f$ radians.

> **Caution**:
> Box2D uses radians, not degrees.

## Changing the length units
Advanced users may change the length unit by calling `b2SetLengthUnitsPerMeter()`
at application startup. If you keep Box2D in a shared library, you will need
to call this if the shared library is reloaded.

If you change the length units to pixels you will need to decide how many pixels
represent a meter. You will also
need to figure out reasonable values for gravity, density, force, and torque.
One of the benefits of using MKS units for physics simulation is that you
can use real world values to get reasonable results.

It is also harder to get support for using Box2D if you change the unit
system, because values are harder to communicate and may become non-intuitive.

## Ids and Definitions
Fast memory management plays a central role in the design of the Box2D
interface. When you create a world, body, shape or joint, you will receive
a handle called an *id*. These ids are opaque and are passed to various functions
to access the underlying data.

These ids provide some safety. If you use an id after it has been freed you will
usually get an assertion. All ids support 64k generations of safety. All ids
also have a corresponding function you can call to check if it is valid.

When you create a world, body, shape, or joint, you need to provide a definition structure.
These definitions contain all the information needed to build the Box2D object. By using
this approach I can prevent construction errors, keep the number of function parameters
small, provide sensible defaults, and reduce the number of accessors.

Here is an example of body creation:

```c
b2BodyDef bodyDef = b2DefaultBodyDef();
bodyDef.position = (b2Vec2){10.0f, 5.0f};
b2BodyId myBodyId = b2CreateBody(myWorldId, &bodyDef);
```

Notice the body definition is initialize by calling `b2DefaultBodyDef()`. This is needed
because C does not have constructors and zero initialization is not suitable for the definitions
used in Box2D.

Also notice that the body definition is a temporary object that is fully copied into the internal
body data structures. Definitions should usually be created on the stack as temporaries.

This is how a body is destroyed:

```c
b2DestroyBody(myBodyId);
myBodyId = b2_nullBodyId;
```

Notice that the body id is set to null using the constant `b2_nullBodyId`. You should treat
ids as opaque data, however you may zero initialize all Box2D ids and they will be considered
*null*.

Shapes are created in a similar way. For example, here is how a box shape is created:

```c
b2ShapeDef shapeDef = b2DefaultShapeDef();
shapeDef.friction = 0.42f;
b2Polygon box = b2MakeBody(0.5f, 0.25f);
b2ShapeId myShapeId = b2CreateCircleShape(myBodyId, &shapeDef, &box);
```

And the shape may be destroyed as follows:

```c
b2DestroyShape(myShapeId);
myShapeId = b2_nullShapeId;
```

For convenience, Box2D will destroy all shapes on a body when the body is destroyed.
Therefore, you may not need to store the shape id.
