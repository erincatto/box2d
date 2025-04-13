# Collision
Box2D provides geometric types and functions. These include:
- primitives: circles, capsules, segments, and convex polygons
- convex hull and related helper functions
- mass and bounding box computation
- local ray and shape casts
- contact manifolds
- shape distance
- time of impact
- dynamic bounding volume tree
- character movement solver

The collision interface is designed to be usable outside of rigid body simulation.
For example, you can use the dynamic tree for other aspects of your game besides physics.

However, the main purpose of Box2D is to be a rigid body physics
engine. So the collision interface only contains features that are also useful in
the physics simulation.

## Shape Primitives
Shape primitives describe collision geometry and may be used independently of
physics simulation. At a minimum, you should understand how to create
primitives that can be later attached to rigid bodies.

Box2D shape primitives support several operations:
- Test a point for overlap with the primitive
- Perform a ray cast against the primitive
- Compute the primitive's bounding box
- Compute the mass properties of the primitive

### Circles
Circles have a center and radius. Circles are solid.

![Circle](images/circle.svg)

```c
b2Circle circle;
circle.center = (b2Vec2){2.0f, 3.0f};
circle.radius = 0.5f;
```

You can also initialize a circle and other structures inline. This is an equivalent circle:

```c
b2Circle circle = {{2.0f, 3.0f}, 0.5f};
```

### Capsules
Capsules have two center points and a radius. The center points are the centers of two
semicircles that are connected by a rectangle.

![Capsule](images/capsule.svg)

```c
b2Capsule capsule;
capsule.center1 = (b2Vec2){1.0f, 1.0f};
capsule.center2 = (b2Vec2){2.0f, 3.0f};
capsule.radius = 0.25f;
```

### Polygons
Box2D polygons are solid convex polygons. A polygon is convex when all
line segments connecting two points in the interior do not cross any
edge of the polygon. Polygons are solid and never hollow. A polygon must
have 3 or more vertices.

![Convex and Concave Polygons](images/convex_concave.svg)

Polygons vertices are stored with a counter clockwise winding (CCW). We
must be careful because the notion of CCW is with respect to a
right-handed coordinate system with the z-axis pointing out of the
plane. This might turn out to be clockwise on your screen, depending on
your coordinate system conventions.

![Polygon Winding Order](images/winding.svg)

The polygon members are public, but you should use initialization
functions to create a polygon. The initialization functions create
normal vectors and perform validation.

Polygons in Box2D have a maximum of 8 vertices, as controlled by #B2_MAX_POLYGON_VERTICES.
If you have more complex shapes, I recommend to use multiple polygons.

There are a few ways to create polygons. You can attempt to create them manually,
but this is not recommended. Instead there are several functions provided to create them.

For example if you need a square or box you can use these functions:

```c
b2Polygon square = b2MakeSquare(0.5f);
b2Polygon box = b2MakeBox(0.5f, 1.0f);
```

The values provided to these functions are *extents*, which are half-widths or half-heights.
This corresponds with circles and capsules using radii instead of diameters.

Box2D also supports rounded polygons. These are convex polygons with a thick rounded skin.

```c
float radius = 0.25f;
b2Polygon roundedBox = b2MakeRoundedBox(0.5f, 1.0f, radius);
```

If you want a box that is not centered on the body origin, you can use an offset box.

```c
b2Vec2 center = {1.0f, 0.0f};
float angle = b2_pi / 4.0f;
b2Rot rotation = b2MakeRot(angle);
b2Polygon offsetBox = b2MakeOffsetBox(0.5f, 1.0f, center, rotation);
```

If you want a more general convex polygon, you can compute the hull using `b2ComputeHull()`. Then you can
create a polygon from the hull. You can make this rounded as well.

```c
b2Vec2 points[] = {{-1.0f, 0.0f}, {1.0f, 0.0f}, {0.0f, 1.0f}};
b2Hull hull = b2ComputeHull(points, 3);
float radius = 0.1f;
b2Polygon roundedTriangle = b2MakePolygon(&hull, radius);
```

If you have an automatic process for generating convex polygons, you may feed a degenerate set of points to `b2ComputeHull()`. You should check that the hull was created successfully before creating the polygon or you will get an assertion.

```c
b2Hull questionableHull = b2ComputeHull(randomPoints, 8);
if (questionableHull.count == 0)
{
    // handle failure
}
```

Degenerate points may be coincident and/or collinear. For the hull to be viable, the enclosed area must be sufficiently positive.

### Segments
Segments are line segments. Segment
shapes can collide with circles, capsules, and polygons but not with other line segments.
The collision algorithms used by Box2D require that at least
one of two colliding shapes has sufficiently positive area. Segment shapes have no area, so
segment-segment collision is not possible.

```c
b2Segment segment1;
segment1.point1 = (b2Vec2){0.0f, 0.0f};
segment2.point2 = (b2Vec2){1.0f, 0.0f};

// equivalent
b2Segment segment2 = {{0.0f, 0.0f}, {1.0f, 0.0f}};
```

### Ghost Collisions
In many cases a game environment is constructed by connecting several
segment shapes end-to-end. This can give rise to an unexpected artifact
when a polygon slides along the chain of segments. In the figure below there is
 a box colliding with an internal vertex. These *ghost* collisions
are caused when the polygon collides with an internal vertex generating
an internal collision normal.

![Ghost Collision](images/ghost_collision.svg){html: width=30%}

If edge1 did not exist this collision would seem fine. With edge1
present, the internal collision seems like a bug. But normally when
Box2D collides two shapes, it views them in isolation.

`b2ChainSegment` provides a mechanism for eliminating ghost
collisions by storing the adjacent *ghost* vertices. Box2D uses these
ghost vertices to prevent internal collisions.

![Ghost Vertices](images/ghost_vertices.svg){html: width=30%}

The Box2D algorithm for dealing with ghost collisions only supports
one-sided collision. The front face is to the right when looking from the first
vertex towards the second vertex. This matches the counter-clockwise winding order
used by polygons.

### Chain segment
Chain segments use a concept called *ghost vertices* that Box2D can use to eliminate ghost
collisions.

```c
b2ChainSegment chainSegment = {0};
chainSegment.ghost1 = (b2Vec2){1.7f, 0.0f};
chainSegment.segment = (b2Segment){{1.0f, 0.25f}, {0.0f, 0.0f}};
chainSegment.ghost2 = (b2Vec2){-1.7f, 0.4f};
```

These ghost vertices must align with vertices of neighboring chain segments, making them
tedious and error-prone to setup.

Chain segments are not created directly. Instead, you can create chains of line
segments. See `b2ChainDef` and `b2CreateChain()`.

## Geometric Queries
You can perform a geometric queries on a single shape.

### Shape Point Test
You can test a point for overlap with a shape. You provide a transform
for the shape and a world point.

```c
b2Vec2 point = {5.0f, 2.0f};
bool hit = b2PointInCapsule(point, &myCapsule);
```

See also `b2PointInCircle()` and `b2PointInPolygon()`.

### Ray Cast
You can cast a ray at a shape to get the point of first intersection and normal vector.

> **Caution**:
> No hit will register if the ray starts inside a convex shape like a circle or polygon. This is
> consistent with Box2D treating convex shapes as solid. 

```c
b2RayCastInput input = {0};
input.origin = (b2Vec2){0.0f, 0.0f};
input.translation = (b2Vec2){1.0f, 0.0f};
input.maxFraction = 1.0f;

b2CastOutput output = b2RayCastPolygon(&input, &myPolygon);
if (output.hit == true)
{
    // do something
}
```

### Shape Cast
You can also cast a shape at another shape. This uses an abstract way of describing the moving shape. It is represented as a point cloud with a radius. This implies a convex shape even if the input data is not convex. The internal algorithm (GJK) will essentially only use the convex portion.

```c
b2ShapeCastInput input = {0};
input.points[0] = (b2Vec2){1.0f, 0.0f};
input.points[1] = (b2Vec2){2.0f, -3.0f};
input.radius = 0.2f;
input.translation = (b2Vec2){1.0f, 0.0f};
input.maxFraction = 1.0f;

b2CastOutput output = b2ShapeCastPolygon(&input, &myPolygon);
if (output.hit == true)
{
    // do something
}
```

Even more generic, you can use `b2ShapeCast()` to linearly cast one point cloud at another point cloud. All shape cast functions use this internally.

### Distance
`b2ShapeDistance()` function can be used to compute the distance between two
shapes. The distance function needs both shapes to be converted into a
`b2DistanceProxy` (which are point clouds with radii). There is also some caching used to warm start the
distance function for repeated calls. This can improve performance when the shapes move by small amounts.

![Distance Function](images/distance.svg)

### Time of Impact
If two shapes are moving fast, they may *tunnel* through each other in a
single time step.

![Tunneling](images/tunneling2.svg){html: width=30%}

The `b2TimeOfImpact()` function is used to determine the time when two moving shapes collide.
This is called the *time of impact* (TOI). The main purpose of `b2TimeOfImpact()` is for
tunnel prevention. Box2D uses this internally to prevent moving objects from tunneling through
static shapes.

The `b2TimeOfImpact()` identifies an initial separating axis and
ensures the shapes do not cross on that axis. This process is repeated
as shapes are moved closer together, until they touch or pass by each other.

The TOI function might miss collisions that are clear at the final positions.
Nevertheless, it is very fast and adequate for tunnel prevention.

![Captured Collision](images/captured_toi.svg){html: width=30%}

![Missed Collision](images/missed_toi.svg){html: width=30%}

It is difficult to put a restriction on the rotation magnitude. There
may be cases where collisions are missed for small rotations. Normally,
these missed rotational collisions should not harm game play. They tend
to be glancing collisions.

The function requires two shapes (converted to `b2DistanceProxy`) and two
`b2Sweep` structures. The sweep structure defines the initial and final
transforms of the shapes.

You can use fixed rotations to perform a *shape cast*. In this case, the
time of impact function will not miss any collisions.

### Contact Manifolds
Box2D has functions to compute contact points for overlapping shapes. If
we consider circle-circle or circle-polygon, we can only get one contact
point and normal. In the case of polygon-polygon we can get two points.
These points share the same normal vector so Box2D groups them into a
manifold structure. The contact solver takes advantage of this to
improve stacking stability.

![Contact Manifold](images/manifolds.svg)

Normally you don't need to compute contact manifolds directly, however
you will likely use the results produced in the simulation.

The `b2Manifold` structure holds a normal vector and up to two contact
points. The contact points store the normal and tangential (friction) impulses
computed in the rigid body simulation.

## Dynamic Tree
`b2DynamicTree` is used by Box2D to organize large numbers of
shapes efficiently. The object does not know directly about shapes. Instead it
operates on axis-aligned bounding boxes (`b2AABB`) with user data integers.

The dynamic tree is a hierarchical AABB tree. Each internal node in the
tree has two children. A leaf node is a single user AABB. The tree uses
rotations to keep the tree balanced, even in the case of degenerate
input.

The tree structure allows for efficient ray casts and region queries.
For example, you may have hundreds of shapes in your scene. You could
perform a ray cast against the scene in a brute force manner by ray
casting each shape. This would be inefficient because it does not take
advantage of shapes being spread out. Instead, you can maintain a
dynamic tree and perform ray casts against the tree. This traverses the
ray through the tree skipping large numbers of shapes.

A region query uses the tree to find all leaf AABBs that overlap a query
AABB. This is faster than a brute force approach because many shapes can
be skipped.

![Ray-cast](images/raycast.svg){html: width=30%}

![Overlap Test](images/overlap_test.svg){html: width=30%}

Normally you will not use the dynamic tree directly. Rather you will go
through the `b2World` functions for ray casts and region queries. If you plan
to instantiate your own dynamic tree, you can learn how to use it by
looking at how Box2D uses it. Also see the `DynamicTree` sample.
