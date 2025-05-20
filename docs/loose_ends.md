# Loose Ends

## User Data
Bodies, shapes, and joints allow you to attach user data
as a `void*`. This is handy when you are examining Box2D data
structures and you want to determine how they relate to the objects in
your game engine.

For example, it is typical to attach an entity pointer to the rigid body
on that entity. This sets up a circular reference. If you have the entity,
you can get the body. If you have the body, you can get the entity.

```c
GameEntity* entity = GameCreateEntity();
b2BodyDef bodyDef = b2DefaultBodyDef();
bodyDef.userData = entity;
entity->bodyId = b2CreateBody(myWorldId, &bodyDef);
```

Here are some examples of cases where you would need the user data:
-   Applying damage to an entity using a collision result.
-   Playing a scripted event if the player is inside an axis-aligned box.
-   Accessing a game structure when Box2D notifies you that a joint is
    going to be destroyed.

Keep in mind that user data is optional and you can put anything in it.
However, you should be consistent. For example, if you want to store an
entity pointer on one body, you should keep an entity pointer on all
bodies. Don't store a `GameEntity` pointer on one body, and a `ParticleSystem`
pointer on another body. Casting a `GameEntity` to a `ParticleSystem` pointer
may lead to a crash.

## Pixels and Coordinate Systems
I recommend using MKS (meters, kilograms, and seconds) units and
radians for angles. You may have trouble working with meters because
your game is expressed in terms of pixels. To deal with this in the
sample I have the whole *game* world in meters and just use an OpenGL
viewport transformation to scale the world into screen space.

You use code like this to scale your graphics.

```c
float lowerX = -25.0f, upperX = 25.0f, lowerY = -5.0f, upperY = 25.0f;
gluOrtho2D(lowerX, upperX, lowerY, upperY);
```

If your game must work in pixel units then you could convert your
length units from pixels to meters when passing values from Box2D.
Likewise you should convert the values received from Box2D from meters
to pixels. This will improve the stability of the physics simulation.

You have to come up with a reasonable conversion factor. I suggest
making this choice based on the size of your characters. Suppose you
have determined to use 50 pixels per meter (because your character is 75
pixels tall). Then you can convert from pixels to meters using these
formulas:

```cpp
xMeters = 0.02f * xPixels;
yMeters = 0.02f * yPixels;
```

In reverse:

```cpp
xPixels = 50.0f * xMeters;
yPixels = 50.0f * yMeters;
```

You should consider using MKS units in your game code and just convert
to pixels when you render. This will simplify your game logic and reduce
the chance for errors since the rendering conversion can be isolated to
a small amount of code.

If you use a conversion factor, you should try tweaking it globally to
make sure nothing breaks. You can also try adjusting it to improve
stability.

If this conversion is not possible, you can set the length units used
by Box2D using `b2SetLengthUnitsPerMeter()`. This is experimental and not
well tested.

## Debug Drawing
You can implement the function pointers in `b2DebugDraw` struct to get detailed
drawing of the Box2D world. Debug draw provides:
- shapes
- joints
- broad-phase axis-aligned bounding boxes (AABBs)
- center of mass
- contact points

This is the preferred method of drawing the Box2D simulation, rather
than accessing the data directly. The reason is that much of the
necessary data is internal and subject to change.

The samples application draws the Box2D world using the `b2DebugDraw`.

## Limitations
Box2D uses several approximations to simulate rigid body physics
efficiently. This brings some limitations.

Here are the current limitations:
1. Extreme mass ratios may cause joint stretching and collision overlap.
2. Box2D uses soft constraints to improve robustness. This can lead to joint and contact flexing.
3. Continuous collision does not handle all situations. For example, general dynamic versus dynamic continuous collision is not handled. [Bullets](#bullets) handle this in a limited way. This is done for performance reasons.
4. Continuous collision does not handle joints. So you may see joint stretching on fast moving objects. Usually the joints recover after a few time steps.
5. Box2D uses the [semi-implicit Euler method](https://en.wikipedia.org/wiki/Semi-implicit_Euler_method) to solve the [equations of motion](https://en.wikipedia.org/wiki/Equations_of_motion). It does not reproduce exactly the parabolic motion of projectiles and has only first-order accuracy. However it is fast and has good stability.
6. Box2D uses the [Gauss-Seidel method](https://en.wikipedia.org/wiki/Gauss%E2%80%93Seidel_method) to solve constraints and achieve real-time performance. You will not get precisely rigid collisions or pixel perfect accuracy. Increasing the sub-step count will improve accuracy.
