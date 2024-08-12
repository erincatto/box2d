# Hello Box2D {#hello}
In the distribution of Box2D is a Hello World unit test written in C. The test
creates a large ground box and a small dynamic box. This code does not
contain any graphics. All you will see is text output in the console of
the box's position over time.

This is a good example of how to get up and running with Box2D.

## Creating a World
Every Box2D program begins with the creation of a world object.
The world is the physics hub that manages memory, objects, and simulation.
The world is represented by an opaque handle called `b2WorldId`.

It is easy to create a Box2D world. First, I create the world definition:

```c
b2WorldDef worldDef = b2DefaultWorldDef();
```

The world definition is a temporary object that you can create on the stack. The function
`b2DefaultWorldDef()` populates the world definition with default values. This is necessary because C does not have constructors and zero initialization is not appropriate for `b2WorldDef`.

Now I configure the world gravity vector. Note that Box2D has no concept of *up* and you may point gravity in any direction you like. Box2D example code uses the positive y-axis as the up direction.

```c
worldDef.gravity = (b2Vec2){0.0f, -10.0f};
```

Now I create the world object.

```c
b2WorldId worldId = b2CreateWorld(&worldDef);
```

The world creation copies all the data it needs out of the world definition, so the world
definition is no longer needed.

So now we have our physics world, let's start adding some stuff to it.

## Creating a Ground Box
Bodies are built using the following steps:
1. Define a body with position, damping, etc.
2. Use the world id to create the body.
3. Define shapes with friction, density, etc.
4. Create shapes on the body.

For step 1 I create the ground body. For this I need a body
definition. With the body definition I specify the initial position of
the ground body.

```c
b2BodyDef groundBodyDef = b2DefaultBodyDef();
groundBodyDef.position = (b2Vec2){0.0f, -10.0f};
```

For step 2 the body definition and the world id are used to create
the ground body. Again, the definition is fully copied and may leave scope after
the body is created. Bodies are static by default. Static bodies don't collide
with other static bodies and are immovable by the simulation.

```c
b2BodyId groundId = b2CreateBody(worldId, &groundBodyDef);
```

Notice that `worldId` is passed by value. Ids are small structures that should
be passed by value.

For step 3 I create a ground polygon. I use the `b2MakeBox()` helper function to
form the ground polygon into a box shape, with the box centered on the
origin of the parent body.

```c
b2Polygon groundBox = b2MakeBox(50.0f, 10.0f);
```

The `b2MakeBox()` function takes the **half-width** and
**half-height** (extents). So in this case the ground box is 100
units wide (x-axis) and 20 units tall (y-axis). Box2D is tuned for
meters, kilograms, and seconds. So you can consider the extents to be in
meters. Box2D generally works best when objects are the size of typical
real world objects. For example, a barrel is about 1 meter tall. Due to
the limitations of floating point arithmetic, using Box2D to model the
movement of glaciers or dust particles is not a good idea.

I'll finish the ground body in step 4 by creating the shape. For this step
I need to create a shape definition which works fine with the default value.

```c
b2ShapeDef groundShapeDef = b2DefaultShapeDef();
b2CreatePolygonShape(groundId, &groundShapeDef, &groundBox);
```

Box2D does not keep a reference to the shape data. It copies the data into the internal
data structures.

Note that every shape must have a parent body, even shapes that are
static. You may attach multiple shapes to a single parent body.

When you attach a shape, the shape's
coordinates become local to the body. So when the body moves, so does
the shape. A shape's world transform is inherited from the parent
body. A shape does not have a transform independent of the body. So we
don't move a shape around on the body. Moving or modifying a shape that
is on a body is possible with certain functions, but it should not be part
of normal simulation. The reason is simple: a body with
morphing shapes is not a rigid body, but Box2D is a rigid body engine.
Many of the algorithms in Box2D are based on the rigid body model.
If this is violated you may get unexpected behavior.

## Creating a Dynamic Body
I can use the same technique to create a
dynamic body. The main difference, besides dimensions, is that I must
establish the dynamic body's mass properties.

First I create the body using CreateBody. By default bodies are static,
so I should set the `b2BodyType` at creation time to make the body
dynamic. I should also use the body definition to put the body at the
intended position for simulation. Creating a body then moving it afterwards is
very inefficient and may cause lag spikes, especially if many bodies are created at
the origin.

```c
b2BodyDef bodyDef = b2DefaultBodyDef();
bodyDef.type = b2_dynamicBody;
bodyDef.position = (b2Vec2){0.0f, 4.0f};
b2BodyId bodyId = b2CreateBody(worldId, &bodyDef);
```

> **Caution**:
> You must set the body type to `b2_dynamicBody` if you want the body to
> move in response to forces (such as gravity).

Next I create and attach a polygon shape using a shape definition.
First I create another box shape:

```c
b2Polygon dynamicBox = b2MakeBox(1.0f, 1.0f);
```

Next I create a shape definition for the box. Notice that I set
density to 1. The default density is 1, so this is unnecessary. Also,
the friction on the shape is set to 0.3.

```c
b2ShapeDef shapeDef = b2DefaultShapeDef();
shapeDef.density = 1.0f;
shapeDef.friction = 0.3f;
```

> **Caution**:
> A dynamic body should have at least one shape with a non-zero density.
> Otherwise you will get strange behavior.

Using the shape definition I can now create the shape. This
automatically updates the mass of the body. You can add as many shapes
as you like to a body. Each one contributes to the total mass.

```c
b2CreatePolygonShape(bodyId, &shapeDef, &dynamicBox);
```

That's it for initialization. We are now ready to begin simulating.

## Simulating the World
I have initialized the ground box and a dynamic box. Now we are
ready to set Newton loose to do his thing. I just have a couple more
issues to consider.

Box2D uses a computational algorithm called an integrator. Integrators
simulate the physics equations at discrete points of time. This goes
along with the traditional game loop where we essentially have a flip
book of movement on the screen. So we need to pick a time step for
Box2D. Generally physics engines for games like a time step at least as
fast as 60Hz or 1/60 seconds. You can get away with larger time steps,
but you will have to be more careful about setting up your simulation.
It is also not good for the time step to vary from frame to frame. A
variable time step produces variable results, which makes it difficult
to debug. So don't tie the time step to your frame rate. Without further ado,
here is the time step.

```c
float timeStep = 1.0f / 60.0f;
```

In addition to the integrator, Box2D also uses a larger bit of code
called a constraint solver. The constraint solver solves all the
constraints in the simulation, one at a time. A single constraint can be
solved perfectly. However, when Box2D solves one constraint, it slightly
disrupts other constraints. To get a good solution, Box2D needs to iterate
over all constraints a number of times.

Box2D uses sub-stepping as a means of constraint iteration. It lets the
simulation move forward in time by small amounts and each constraint
gets a chance to react to the changes.

The suggested sub-step count for Box2D is 4. You can tune this number
to your liking, just keep in mind that this has a trade-off between
performance and accuracy. Using fewer sub-steps increases performance
but accuracy suffers. Likewise, using
more sub-steps decreases performance but improves the quality of your
simulation. For this example, I will use 4 sub-steps.

```c
int subStepCount = 4;
```

Note that the time step and the sub-step count are related. As the time-step
decreases, the size of the sub-steps also decreases. For example, at 60Hz
time step and 4 sub-steps, the sub-steps operate at 240Hz. With 8 sub-steps
the sub-step is 480Hz!

We are now ready to begin the simulation loop. In your game the
simulation loop can be merged with your game loop. In each pass through
your game loop you call `b2World_Step()`. Just one call is usually enough,
depending on your frame rate and your physics time step. I recommend this article
[Fix Your Timestep!](https://gafferongames.com/post/fix_your_timestep/) to run
your game simulation at a fixed rate.

The Hello World test was designed to be simple, so it has no
graphical output. The code prints out the position and rotation of the
dynamic body. Here is the simulation loop that simulates 90 time steps
for a total of 1.5 seconds of simulated time.

```c
for (int i = 0; i < 90; ++i)
{
	b2World_Step(worldId, timeStep, subStepCount);
    b2Vec2 position = b2Body_GetPosition(bodyId);
    b2Rot rotation = b2Body_GetRotation(bodyId);
    printf("%4.2f %4.2f %4.2f\n", position.x, position.y, b2Rot_GetAngle(rotation));
}
```

The output shows the box falling and landing on the ground box. Your
output should look like this:

```
0.00 4.00 0.00
0.00 3.99 0.00
0.00 3.98 0.00
...
0.00 1.25 0.00
0.00 1.13 0.00
0.00 1.01 0.00
```

## Cleanup
When you are done with the simulation, you should destroy the world.

```c
b2DestroyWorld(worldId);
```

This efficiently destroys all bodies, shapes, and joints in the simulation.
