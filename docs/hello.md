# Hello Box2D
In the distribution of Box2D is a Hello World project. The program
creates a large ground box and a small dynamic box. This code does not
contain any graphics. All you will see is text output in the console of
the box's position over time.

This is a good example of how to get up and running with Box2D.

## Creating a World
Every Box2D program begins with the creation of a b2World object.
b2World is the physics hub that manages memory, objects, and simulation.
You can allocate the physics world on the stack, heap, or data section.

It is easy to create a Box2D world. First, we define the gravity vector.

```cpp
b2Vec2 gravity(0.0f, -10.0f);
```

Now we create the world object. Note that we are creating the world on
the stack, so the world must remain in scope.

```cpp
b2World world(gravity);
```

So now we have our physics world, let's start adding some stuff to it.

## Creating a Ground Box
Bodies are built using the following steps:
1. Define a body with position, damping, etc.
2. Use the world object to create the body.
3. Define fixtures with a shape, friction, density, etc.
4. Create fixtures on the body.

For step 1 we create the ground body. For this we need a body
definition. With the body definition we specify the initial position of
the ground body.

```cpp
b2BodyDef groundBodyDef;
groundBodyDef.position.Set(0.0f, -10.0f);
```

For step 2 the body definition is passed to the world object to create
the ground body. The world object does not keep a reference to the body
definition. Bodies are static by default. Static bodies don't collide
with other static bodies and are immovable.

```cpp
b2Body* groundBody = world.CreateBody(&groundBodyDef);
```

For step 3 we create a ground polygon. We use the SetAsBox shortcut to
form the ground polygon into a box shape, with the box centered on the
origin of the parent body.

```cpp
b2PolygonShape groundBox;
groundBox.SetAsBox(50.0f, 10.0f);
```

The SetAsBox function takes the **half**-**width** and
**half**-**height** (extents). So in this case the ground box is 100
units wide (x-axis) and 20 units tall (y-axis). Box2D is tuned for
meters, kilograms, and seconds. So you can consider the extents to be in
meters. Box2D generally works best when objects are the size of typical
real world objects. For example, a barrel is about 1 meter tall. Due to
the limitations of floating point arithmetic, using Box2D to model the
movement of glaciers or dust particles is not a good idea.

We finish the ground body in step 4 by creating the shape fixture. For
this step we have a shortcut. We do not have a need to alter the default
fixture material properties, so we can pass the shape directly to the
body without creating a fixture definition. Later we will see how to use
a fixture definition for customized material properties. The second
parameter is the shape density in kilograms per meter squared. A static
body has zero mass by definition, so the density is not used in this
case.

```cpp
groundBody->CreateFixture(&groundBox, 0.0f);
```

Box2D does not keep a reference to the shape. It clones the data into a
new b2Shape object.

Note that every fixture must have a parent body, even fixtures that are
static. However, you can attach all static fixtures to a single static
body.

When you attach a shape to a body using a fixture, the shape's
coordinates become local to the body. So when the body moves, so does
the shape. A fixture's world transform is inherited from the parent
body. A fixture does not have a transform independent of the body. So we
don't move a shape around on the body. Moving or modifying a shape that
is on a body is not supported. The reason is simple: a body with
morphing shapes is not a rigid body, but Box2D is a rigid body engine.
Many of the assumptions made in Box2D are based on the rigid body model.
If this is violated many things will break

## Creating a Dynamic Body
So now we have a ground body. We can use the same technique to create a
dynamic body. The main difference, besides dimensions, is that we must
establish the dynamic body's mass properties.

First we create the body using CreateBody. By default bodies are static,
so we should set the b2BodyType at construction time to make the body
dynamic.

```cpp
b2BodyDef bodyDef;
bodyDef.type = b2_dynamicBody;
bodyDef.position.Set(0.0f, 4.0f);
b2Body* body = world.CreateBody(&bodyDef);
```

> **Caution**:
> You must set the body type to b2_dynamicBody if you want the body to
> move in response to forces.

Next we create and attach a polygon shape using a fixture definition.
First we create a box shape:

```cpp
b2PolygonShape dynamicBox;
dynamicBox.SetAsBox(1.0f, 1.0f);
```

Next we create a fixture definition using the box. Notice that we set
density to 1. The default density is zero. Also, the friction on the
shape is set to 0.3.

```cpp
b2FixtureDef fixtureDef;
fixtureDef.shape = &dynamicBox;
fixtureDef.density = 1.0f;
fixtureDef.friction = 0.3f;
```

> **Caution**:
> A dynamic body should have at least one fixture with a non-zero density.
> Otherwise you will get strange behavior.

Using the fixture definition we can now create the fixture. This
automatically updates the mass of the body. You can add as many fixtures
as you like to a body. Each one contributes to the total mass.

```cpp
body->CreateFixture(&fixtureDef);
```

That's it for initialization. We are now ready to begin simulating.

## Simulating the World
So we have initialized the ground box and a dynamic box. Now we are
ready to set Newton loose to do his thing. We just have a couple more
issues to consider.

Box2D uses a computational algorithm called an integrator. Integrators
simulate the physics equations at discrete points of time. This goes
along with the traditional game loop where we essentially have a flip
book of movement on the screen. So we need to pick a time step for
Box2D. Generally physics engines for games like a time step at least as
fast as 60Hz or 1/60 seconds. You can get away with larger time steps,
but you will have to be more careful about setting up the definitions
for your world. We also don't like the time step to change much. A
variable time step produces variable results, which makes it difficult
to debug. So don't tie the time step to your frame rate (unless you
really, really have to). Without further ado, here is the time step.

```cpp
float timeStep = 1.0f / 60.0f;
```

In addition to the integrator, Box2D also uses a larger bit of code
called a constraint solver. The constraint solver solves all the
constraints in the simulation, one at a time. A single constraint can be
solved perfectly. However, when we solve one constraint, we slightly
disrupt other constraints. To get a good solution, we need to iterate
over all constraints a number of times.

There are two phases in the constraint solver: a velocity phase and a
position phase. In the velocity phase the solver computes the impulses
necessary for the bodies to move correctly. In the position phase the
solver adjusts the positions of the bodies to reduce overlap and joint
detachment. Each phase has its own iteration count. In addition, the
position phase may exit iterations early if the errors are small.

The suggested iteration count for Box2D is 8 for velocity and 3 for
position. You can tune this number to your liking, just keep in mind
that this has a trade-off between performance and accuracy. Using fewer
iterations increases performance but accuracy suffers. Likewise, using
more iterations decreases performance but improves the quality of your
simulation. For this simple example, we don't need much iteration. Here
are our chosen iteration counts.

```cpp
int32 velocityIterations = 6;
int32 positionIterations = 2;
```

Note that the time step and the iteration count are completely
unrelated. An iteration is not a sub-step. One solver iteration is a
single pass over all the constraints within a time step. You can have
multiple passes over the constraints within a single time step.

We are now ready to begin the simulation loop. In your game the
simulation loop can be merged with your game loop. In each pass through
your game loop you call b2World::Step. Just one call is usually enough,
depending on your frame rate and your physics time step.

The Hello World program was designed to be simple, so it has no
graphical output. The code prints out the position and rotation of the
dynamic body. Here is the simulation loop that simulates 60 time steps
for a total of 1 second of simulated time.

```cpp
for (int32 i = 0; i < 60; ++i)
{
    world.Step(timeStep, velocityIterations, positionIterations);
    b2Vec2 position = body->GetPosition();
    float angle = body->GetAngle();
    printf("%4.2f %4.2f %4.2f\n", position.x, position.y, angle);
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
When a world leaves scope or is deleted by calling delete on a pointer,
all the memory reserved for bodies, fixtures, and joints is freed. This
is done to improve performance and make your life easier. However, you
will need to nullify any body, fixture, or joint pointers you have
because they will become invalid.
