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
