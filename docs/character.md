# Character mover

> **Caution**:
> The character mover feature is new to version 3.1 and should be considered experimental.

Box2D provides a few structures and functions you can use to build a character mover.

These features support a `geometric` character mover. This is like
a `kinematic` mover except the character mover is not a rigid body and does
not exist in the simulation world. This is done to achieve features that
would be difficult with a kinematic body.

This type of mover may not be suitable for your game. It is less physical than a rigid body, but
it gives you more control over the movement. It is the type of mover you might find
in a first-person shooter or a game with platforming elements.

The mover is assumed to be a capsule. Using a capsule helps keep movement smooth. The capsule should
have a significant radius. It does not have to be a vertical capsule, but that is likely
to be the easiest setup. There is no explicit handling of rotation. But slow rotation can
work with this system.

Let's review the features. First there are a couple world query functions.

`b2World_CastMover()` is a custom shape cast that tries to avoid getting stuck when shapes start out touching.
The feature is called _encroachment_. Since the capsule has a significant radius it can move closer
to a surface it is touching without the inner line segment generating an overlap, which would cause
the shape cast to fail. Due to the internal use of GJK, encroachment has little cost. The idea with encroachment is that
the mover is trying to slide along a surface and we don't want to stop that even if there is some small movement into the surface.

`b2World_CollideMover()` complements the cast function. This function generates collision planes for touching and/or overlapped surfaces. The character mover is assumed to have a fixed rotation, so it doesn't need contact manifolds or contact points. It just needs collision planes. Each plane is returned with the `b2Plane` and a `b2ShapeId` for each shape the mover is touching.

Once you have some collision planes from `b2World_CollideMover()`, you can process and filter them to generate an array of `b2CollisionPlane`. These collision planes can then be sent to `b2SolvePlanes()` to generate a new position for the mover
that attempts to find the optimal new position given the current position.

These collision planes support *soft collision* using a `pushLimit`. This push limit is a distance value. A rigid surface will have a push limit of `FLT_MAX`. However, you may want some surfaces to have a limited effect on the character. For example, you may want the mover to push through other players or enemies yet still resolve the collision so they are not overlapped. Another example is a door or elevator that could otherwise push the mover through the floor.

Finally after calling `b2SolverPlanes()` you can call `b2ClipVector()` to clip your velocity vector so the mover will not keep trying to push into a wall, which could lead to a huge velocity accumulation otherwise.

The `Mover` sample shows all these functions being used together. It also includes other common character features such as acceleration and friction, jumping, and a pogo stick.

![Character Mover](images/mover.png)
