# Loose Ends

## User data

Bodies, shapes, and joints allow you to attach user data as a `void*`. This is handy when you are
examining Box2D data structures and you want to determine how they relate to the objects in your
game engine.

For example, it is typical to attach an entity pointer to the rigid body on that entity. This sets
up a circular reference. If you have the entity, you can get the body. If you have the body, you can
get the entity.

```c
GameEntity* entity = GameCreateEntity();
b2BodyDef bodyDef = b2DefaultBodyDef();
bodyDef.userData = entity;
entity->bodyId = b2CreateBody(myWorldId, &bodyDef);
```

Here are some examples of cases where you would need the user data:

- Applying damage to an entity using a collision result.
- Playing a scripted event if the player is inside an axis-aligned box.
- Accessing a game structure from a body movement event.

Keep in mind that user data is optional and you can put anything in it. However, you should be
consistent. For example, if you want to store an entity pointer on one body, you should keep an
entity pointer on all bodies. Don't store a `GameEntity` pointer on one body, and a `ParticleSystem`
pointer on another body. Casting a `GameEntity` to a `ParticleSystem` pointer may lead to a crash.

## Pixels and coordinate systems

I recommend using MKS (meters, kilograms, and seconds) units and radians for angles. You may have
trouble working with meters because your game is expressed in terms of pixels. To deal with this in
the sample I have the whole *game* world in meters and just use an projection transformation in the
shader to scale the world into screen space. Read this
[article](https://www.songho.ca/opengl/gl_projectionmatrix.html) for details on projection matrices.

If your game must work in pixel units then use `b2SetLengthUnitsPerMeter()` to adjust the tolerances
Box2D uses internally. Call the function once at application startup and before creating a Box2D
world.

```c
float pixelsPerMeter = 32.0f;
b2SetLengthUnitsPerMeter(pixelsPerMeter);
```

## Debug drawing

You can implement the function pointers in the `b2DebugDraw` struct to get detailed drawing of the
Box2D world. Debug draw provides:

- shapes
- joints
- broad-phase axis-aligned bounding boxes (AABBs)
- center of mass
- contact points

This is the preferred method of drawing the Box2D simulation, rather than accessing the data
directly. The reason is that much of the necessary data is internal and subject to change.

The samples application draws the Box2D world using `b2DebugDraw`.

## Limitations

Box2D uses several approximations to simulate rigid body physics efficiently. This brings some
limitations.

Here are the current limitations:

1. Extreme mass ratios may cause joint stretching and collision overlap.
2. Box2D uses soft constraints to improve robustness. This can lead to joint and contact flexing.
3. Continuous collision does not handle all situations. For example, general dynamic versus dynamic
   continuous collision is not handled. [Bullets](@ref bullets) handle this in a limited way. This
   is done for performance reasons.
4. Continuous collision does not handle joints. So you may see joint stretching on fast moving
   objects. Usually the joints recover after a few time steps.
5. Box2D uses the [semi-implicit Euler
   method](https://en.wikipedia.org/wiki/Semi-implicit_Euler_method) to solve the [equations of
   motion](https://en.wikipedia.org/wiki/Equations_of_motion). It does not reproduce exactly the
   parabolic motion of projectiles and has only first-order accuracy. However it is fast and has
   good stability.
6. Box2D uses the [Gauss-Seidel method](https://en.wikipedia.org/wiki/Gauss%E2%80%93Seidel_method)
   to solve constraints and achieve real-time performance. You will not get precisely rigid
   collisions or pixel perfect accuracy. Increasing the sub-step count will improve accuracy.
