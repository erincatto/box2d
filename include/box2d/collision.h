// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "base.h"
#include "math_functions.h"

#include <stdbool.h>

typedef struct b2Circle b2Circle;
typedef struct b2Capsule b2Capsule;
typedef struct b2DistanceCache b2DistanceCache;
typedef struct b2Polygon b2Polygon;
typedef struct b2Segment b2Segment;
typedef struct b2ChainSegment b2ChainSegment;

typedef struct b2Hull b2Hull;

/**
 * @defgroup geometry Geometry
 * @brief Geometry types and algorithms
 *
 * Definitions of circles, capsules, segments, and polygons. Various algorithms to compute hulls, mass properties, and so on.
 * @{
 */

/// The maximum number of vertices on a convex polygon. Changing this affects performance even if you
/// don't use more vertices.
#define b2_maxPolygonVertices 8

/// Low level ray cast input data
typedef struct b2RayCastInput
{
	/// Start point of the ray cast
	b2Vec2 origin;

	/// Translation of the ray cast
	b2Vec2 translation;

	/// The maximum fraction of the translation to consider, typically 1
	float maxFraction;
} b2RayCastInput;

/// Low level shape cast input in generic form. This allows casting an arbitrary point
/// cloud wrap with a radius. For example, a circle is a single point with a non-zero radius.
/// A capsule is two points with a non-zero radius. A box is four points with a zero radius.
typedef struct b2ShapeCastInput
{
	/// A point cloud to cast
	b2Vec2 points[b2_maxPolygonVertices];

	/// The number of points
	int32_t count;

	/// The radius around the point cloud
	float radius;

	/// The translation of the shape cast
	b2Vec2 translation;

	/// The maximum fraction of the translation to consider, typically 1
	float maxFraction;
} b2ShapeCastInput;

/// Low level ray cast or shape-cast output data
typedef struct b2CastOutput
{
	/// The surface normal at the hit point
	b2Vec2 normal;

	/// The surface hit point
	b2Vec2 point;

	/// The fraction of the input translation at collision
	float fraction;

	/// The number of iterations used
	int32_t iterations;

	/// Did the cast hit?
	bool hit;
} b2CastOutput;

/// This holds the mass data computed for a shape.
typedef struct b2MassData
{
	/// The mass of the shape, usually in kilograms.
	float mass;

	/// The position of the shape's centroid relative to the shape's origin.
	b2Vec2 center;

	/// The rotational inertia of the shape about the local origin.
	float rotationalInertia;
} b2MassData;

/// A solid circle
typedef struct b2Circle
{
	/// The local center
	b2Vec2 center;

	/// The radius
	float radius;
} b2Circle;

/// A solid capsule can be viewed as two semicircles connected
/// by a rectangle.
typedef struct b2Capsule
{
	/// Local center of the first semicircle
	b2Vec2 center1;

	/// Local center of the second semicircle
	b2Vec2 center2;

	/// The radius of the semicircles
	float radius;
} b2Capsule;

/// A solid convex polygon. It is assumed that the interior of the polygon is to
/// the left of each edge.
/// Polygons have a maximum number of vertices equal to b2_maxPolygonVertices.
/// In most cases you should not need many vertices for a convex polygon.
/// @warning DO NOT fill this out manually, instead use a helper function like
/// b2MakePolygon or b2MakeBox.
typedef struct b2Polygon
{
	/// The polygon vertices
	b2Vec2 vertices[b2_maxPolygonVertices];

	/// The outward normal vectors of the polygon sides
	b2Vec2 normals[b2_maxPolygonVertices];

	/// The centroid of the polygon
	b2Vec2 centroid;

	/// The external radius for rounded polygons
	float radius;

	/// The number of polygon vertices
	int32_t count;
} b2Polygon;

/// A line segment with two-sided collision.
typedef struct b2Segment
{
	/// The first point
	b2Vec2 point1;

	/// The second point
	b2Vec2 point2;
} b2Segment;

/// A line segment with one-sided collision. Only collides on the right side.
/// Several of these are generated for a chain shape.
/// ghost1 -> point1 -> point2 -> ghost2
typedef struct b2ChainSegment
{
	/// The tail ghost vertex
	b2Vec2 ghost1;

	/// The line segment
	b2Segment segment;

	/// The head ghost vertex
	b2Vec2 ghost2;

	/// The owning chain shape index (internal usage only)
	int32_t chainId;
} b2ChainSegment;

/// Validate ray cast input data (NaN, etc)
B2_API bool b2IsValidRay( const b2RayCastInput* input );

/// Make a convex polygon from a convex hull. This will assert if the hull is not valid.
/// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
B2_API b2Polygon b2MakePolygon( const b2Hull* hull, float radius );

/// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
/// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
B2_API b2Polygon b2MakeOffsetPolygon( const b2Hull* hull, b2Vec2 position, b2Rot rotation );

/// Make an offset convex polygon from a convex hull. This will assert if the hull is not valid.
/// @warning Do not manually fill in the hull data, it must come directly from b2ComputeHull
B2_API b2Polygon b2MakeOffsetRoundedPolygon( const b2Hull* hull, b2Vec2 position, b2Rot rotation, float radius );

/// Make a square polygon, bypassing the need for a convex hull.
/// @param h the half-width
B2_API b2Polygon b2MakeSquare( float h );

/// Make a box (rectangle) polygon, bypassing the need for a convex hull.
/// @param hx the half-width
/// @param hy the half-height
B2_API b2Polygon b2MakeBox( float hx, float hy );

/// Make a rounded box, bypassing the need for a convex hull.
/// @param hx the half-width
/// @param hy the half-height
/// @param radius the radius of the rounded extension
B2_API b2Polygon b2MakeRoundedBox( float hx, float hy, float radius );

/// Make an offset box, bypassing the need for a convex hull.
/// @param hx the half-width
/// @param hy the half-height
/// @param center the local center of the box
/// @param rotation the local rotation of the box
B2_API b2Polygon b2MakeOffsetBox( float hx, float hy, b2Vec2 center, b2Rot rotation );

/// Make an offset rounded box, bypassing the need for a convex hull.
/// @param hx the half-width
/// @param hy the half-height
/// @param center the local center of the box
/// @param rotation the local rotation of the box
/// @param radius the radius of the rounded extension
B2_API b2Polygon b2MakeOffsetRoundedBox( float hx, float hy, b2Vec2 center, b2Rot rotation, float radius );

/// Transform a polygon. This is useful for transferring a shape from one body to another.
B2_API b2Polygon b2TransformPolygon( b2Transform transform, const b2Polygon* polygon );

/// Compute mass properties of a circle
B2_API b2MassData b2ComputeCircleMass( const b2Circle* shape, float density );

/// Compute mass properties of a capsule
B2_API b2MassData b2ComputeCapsuleMass( const b2Capsule* shape, float density );

/// Compute mass properties of a polygon
B2_API b2MassData b2ComputePolygonMass( const b2Polygon* shape, float density );

/// Compute the bounding box of a transformed circle
B2_API b2AABB b2ComputeCircleAABB( const b2Circle* shape, b2Transform transform );

/// Compute the bounding box of a transformed capsule
B2_API b2AABB b2ComputeCapsuleAABB( const b2Capsule* shape, b2Transform transform );

/// Compute the bounding box of a transformed polygon
B2_API b2AABB b2ComputePolygonAABB( const b2Polygon* shape, b2Transform transform );

/// Compute the bounding box of a transformed line segment
B2_API b2AABB b2ComputeSegmentAABB( const b2Segment* shape, b2Transform transform );

/// Test a point for overlap with a circle in local space
B2_API bool b2PointInCircle( b2Vec2 point, const b2Circle* shape );

/// Test a point for overlap with a capsule in local space
B2_API bool b2PointInCapsule( b2Vec2 point, const b2Capsule* shape );

/// Test a point for overlap with a convex polygon in local space
B2_API bool b2PointInPolygon( b2Vec2 point, const b2Polygon* shape );

/// Ray cast versus circle in shape local space. Initial overlap is treated as a miss.
B2_API b2CastOutput b2RayCastCircle( const b2RayCastInput* input, const b2Circle* shape );

/// Ray cast versus capsule in shape local space. Initial overlap is treated as a miss.
B2_API b2CastOutput b2RayCastCapsule( const b2RayCastInput* input, const b2Capsule* shape );

/// Ray cast versus segment in shape local space. Optionally treat the segment as one-sided with hits from
/// the left side being treated as a miss.
B2_API b2CastOutput b2RayCastSegment( const b2RayCastInput* input, const b2Segment* shape, bool oneSided );

/// Ray cast versus polygon in shape local space. Initial overlap is treated as a miss.
B2_API b2CastOutput b2RayCastPolygon( const b2RayCastInput* input, const b2Polygon* shape );

/// Shape cast versus a circle. Initial overlap is treated as a miss.
B2_API b2CastOutput b2ShapeCastCircle( const b2ShapeCastInput* input, const b2Circle* shape );

/// Shape cast versus a capsule. Initial overlap is treated as a miss.
B2_API b2CastOutput b2ShapeCastCapsule( const b2ShapeCastInput* input, const b2Capsule* shape );

/// Shape cast versus a line segment. Initial overlap is treated as a miss.
B2_API b2CastOutput b2ShapeCastSegment( const b2ShapeCastInput* input, const b2Segment* shape );

/// Shape cast versus a convex polygon. Initial overlap is treated as a miss.
B2_API b2CastOutput b2ShapeCastPolygon( const b2ShapeCastInput* input, const b2Polygon* shape );

/// A convex hull. Used to create convex polygons.
/// @warning Do not modify these values directly, instead use b2ComputeHull()
typedef struct b2Hull
{
	/// The final points of the hull
	b2Vec2 points[b2_maxPolygonVertices];

	/// The number of points
	int32_t count;
} b2Hull;

/// Compute the convex hull of a set of points. Returns an empty hull if it fails.
/// Some failure cases:
/// - all points very close together
/// - all points on a line
/// - less than 3 points
/// - more than b2_maxPolygonVertices points
/// This welds close points and removes collinear points.
/// @warning Do not modify a hull once it has been computed
B2_API b2Hull b2ComputeHull( const b2Vec2* points, int32_t count );

/// This determines if a hull is valid. Checks for:
/// - convexity
/// - collinear points
/// This is expensive and should not be called at runtime.
B2_API bool b2ValidateHull( const b2Hull* hull );

/**@}*/

/**
 * @defgroup distance Distance
 * Functions for computing the distance between shapes.
 *
 * These are advanced functions you can use to perform distance calculations. There
 * are functions for computing the closest points between shapes, doing linear shape casts,
 * and doing rotational shape casts. The latter is called time of impact (TOI).
 * @{
 */

/// Result of computing the distance between two line segments
typedef struct b2SegmentDistanceResult
{
	/// The closest point on the first segment
	b2Vec2 closest1;

	/// The closest point on the second segment
	b2Vec2 closest2;

	/// The barycentric coordinate on the first segment
	float fraction1;

	/// The barycentric coordinate on the second segment
	float fraction2;

	/// The squared distance between the closest points
	float distanceSquared;
} b2SegmentDistanceResult;

/// Compute the distance between two line segments, clamping at the end points if needed.
B2_API b2SegmentDistanceResult b2SegmentDistance( b2Vec2 p1, b2Vec2 q1, b2Vec2 p2, b2Vec2 q2 );

/// A distance proxy is used by the GJK algorithm. It encapsulates any shape.
typedef struct b2DistanceProxy
{
	/// The point cloud
	b2Vec2 points[b2_maxPolygonVertices];

	/// The number of points
	int32_t count;

	/// The external radius of the point cloud
	float radius;
} b2DistanceProxy;

/// Used to warm start b2Distance. Set count to zero on first call or
/// use zero initialization.
typedef struct b2DistanceCache
{
	/// The number of stored simplex points
	uint16_t count;

	/// The cached simplex indices on shape A
	uint8_t indexA[3];

	/// The cached simplex indices on shape B
	uint8_t indexB[3];
} b2DistanceCache;

static const b2DistanceCache b2_emptyDistanceCache = B2_ZERO_INIT;

/// Input for b2ShapeDistance
typedef struct b2DistanceInput
{
	/// The proxy for shape A
	b2DistanceProxy proxyA;

	/// The proxy for shape B
	b2DistanceProxy proxyB;

	/// The world transform for shape A
	b2Transform transformA;

	/// The world transform for shape B
	b2Transform transformB;

	/// Should the proxy radius be considered?
	bool useRadii;
} b2DistanceInput;

/// Output for b2ShapeDistance
typedef struct b2DistanceOutput
{
	b2Vec2 pointA;		  ///< Closest point on shapeA
	b2Vec2 pointB;		  ///< Closest point on shapeB
	float distance;		  ///< The final distance, zero if overlapped
	int32_t iterations;	  ///< Number of GJK iterations used
	int32_t simplexCount; ///< The number of simplexes stored in the simplex array
} b2DistanceOutput;

/// Simplex vertex for debugging the GJK algorithm
typedef struct b2SimplexVertex
{
	b2Vec2 wA;		///< support point in proxyA
	b2Vec2 wB;		///< support point in proxyB
	b2Vec2 w;		///< wB - wA
	float a;		///< barycentric coordinate for closest point
	int32_t indexA; ///< wA index
	int32_t indexB; ///< wB index
} b2SimplexVertex;

/// Simplex from the GJK algorithm
typedef struct b2Simplex
{
	b2SimplexVertex v1, v2, v3; ///< vertices
	int32_t count;				///< number of valid vertices
} b2Simplex;

/// Compute the closest points between two shapes represented as point clouds.
/// b2DistanceCache cache is input/output. On the first call set b2DistanceCache.count to zero.
/// The underlying GJK algorithm may be debugged by passing in debug simplexes and capacity. You may pass in NULL and 0 for these.
B2_API b2DistanceOutput b2ShapeDistance( b2DistanceCache* cache, const b2DistanceInput* input, b2Simplex* simplexes,
										 int simplexCapacity );

/// Input parameters for b2ShapeCast
typedef struct b2ShapeCastPairInput
{
	b2DistanceProxy proxyA; ///< The proxy for shape A
	b2DistanceProxy proxyB; ///< The proxy for shape B
	b2Transform transformA; ///< The world transform for shape A
	b2Transform transformB; ///< The world transform for shape B
	b2Vec2 translationB;	///< The translation of shape B
	float maxFraction;		///< The fraction of the translation to consider, typically 1
} b2ShapeCastPairInput;

/// Perform a linear shape cast of shape B moving and shape A fixed. Determines the hit point, normal, and translation fraction.
B2_API b2CastOutput b2ShapeCast( const b2ShapeCastPairInput* input );

/// Make a proxy for use in GJK and related functions.
B2_API b2DistanceProxy b2MakeProxy( const b2Vec2* vertices, int32_t count, float radius );

/// This describes the motion of a body/shape for TOI computation. Shapes are defined with respect to the body origin,
/// which may not coincide with the center of mass. However, to support dynamics we must interpolate the center of mass
/// position.
typedef struct b2Sweep
{
	b2Vec2 localCenter; ///< Local center of mass position
	b2Vec2 c1;			///< Starting center of mass world position
	b2Vec2 c2;			///< Ending center of mass world position
	b2Rot q1;			///< Starting world rotation
	b2Rot q2;			///< Ending world rotation
} b2Sweep;

/// Evaluate the transform sweep at a specific time.
B2_API b2Transform b2GetSweepTransform( const b2Sweep* sweep, float time );

/// Input parameters for b2TimeOfImpact
typedef struct b2TOIInput
{
	b2DistanceProxy proxyA; ///< The proxy for shape A
	b2DistanceProxy proxyB; ///< The proxy for shape B
	b2Sweep sweepA;			///< The movement of shape A
	b2Sweep sweepB;			///< The movement of shape B
	float tMax;				///< Defines the sweep interval [0, tMax]
} b2TOIInput;

/// Describes the TOI output
typedef enum b2TOIState
{
	b2_toiStateUnknown,
	b2_toiStateFailed,
	b2_toiStateOverlapped,
	b2_toiStateHit,
	b2_toiStateSeparated
} b2TOIState;

/// Output parameters for b2TimeOfImpact.
typedef struct b2TOIOutput
{
	b2TOIState state; ///< The type of result
	float t;		  ///< The time of the collision
} b2TOIOutput;

/// Compute the upper bound on time before two shapes penetrate. Time is represented as
/// a fraction between [0,tMax]. This uses a swept separating axis and may miss some intermediate,
/// non-tunneling collisions. If you change the time interval, you should call this function
/// again.
B2_API b2TOIOutput b2TimeOfImpact( const b2TOIInput* input );

/**@}*/

/**
 * @defgroup collision Collision
 * @brief Functions for colliding pairs of shapes
 * @{
 */

/// A manifold point is a contact point belonging to a contact
/// manifold. It holds details related to the geometry and dynamics
/// of the contact points.
typedef struct b2ManifoldPoint
{
	/// Location of the contact point in world space. Subject to precision loss at large coordinates.
	/// @note Should only be used for debugging.
	b2Vec2 point;

	/// Location of the contact point relative to bodyA's origin in world space
	/// @note When used internally to the Box2D solver, this is relative to the center of mass.
	b2Vec2 anchorA;

	/// Location of the contact point relative to bodyB's origin in world space
	/// @note When used internally to the Box2D solver, this is relative to the center of mass.
	b2Vec2 anchorB;

	/// The separation of the contact point, negative if penetrating
	float separation;

	/// The impulse along the manifold normal vector.
	float normalImpulse;

	/// The friction impulse
	float tangentImpulse;

	/// The maximum normal impulse applied during sub-stepping
	/// This could be a bool to indicate the point is confirmed (may be a speculative point)
	float maxNormalImpulse;

	/// Relative normal velocity pre-solve. Used for hit events. If the normal impulse is
	/// zero then there was no hit. Negative means shapes are approaching.
	float normalVelocity;

	/// Uniquely identifies a contact point between two shapes
	uint16_t id;

	/// Did this contact point exist the previous step?
	bool persisted;
} b2ManifoldPoint;

/// A contact manifold describes the contact points between colliding shapes
typedef struct b2Manifold
{
	/// The manifold points, up to two are possible in 2D
	b2ManifoldPoint points[2];

	/// The unit normal vector in world space, points from shape A to bodyB
	b2Vec2 normal;

	/// The number of contacts points, will be 0, 1, or 2
	int32_t pointCount;
} b2Manifold;

/// Compute the contact manifold between two circles
B2_API b2Manifold b2CollideCircles( const b2Circle* circleA, b2Transform xfA, const b2Circle* circleB, b2Transform xfB );

/// Compute the contact manifold between a capsule and circle
B2_API b2Manifold b2CollideCapsuleAndCircle( const b2Capsule* capsuleA, b2Transform xfA, const b2Circle* circleB,
											 b2Transform xfB );

/// Compute the contact manifold between an segment and a circle
B2_API b2Manifold b2CollideSegmentAndCircle( const b2Segment* segmentA, b2Transform xfA, const b2Circle* circleB,
											 b2Transform xfB );

/// Compute the contact manifold between a polygon and a circle
B2_API b2Manifold b2CollidePolygonAndCircle( const b2Polygon* polygonA, b2Transform xfA, const b2Circle* circleB,
											 b2Transform xfB );

/// Compute the contact manifold between a capsule and circle
B2_API b2Manifold b2CollideCapsules( const b2Capsule* capsuleA, b2Transform xfA, const b2Capsule* capsuleB, b2Transform xfB );

/// Compute the contact manifold between an segment and a capsule
B2_API b2Manifold b2CollideSegmentAndCapsule( const b2Segment* segmentA, b2Transform xfA, const b2Capsule* capsuleB,
											  b2Transform xfB );

/// Compute the contact manifold between a polygon and capsule
B2_API b2Manifold b2CollidePolygonAndCapsule( const b2Polygon* polygonA, b2Transform xfA, const b2Capsule* capsuleB,
											  b2Transform xfB );

/// Compute the contact manifold between two polygons
B2_API b2Manifold b2CollidePolygons( const b2Polygon* polygonA, b2Transform xfA, const b2Polygon* polygonB, b2Transform xfB );

/// Compute the contact manifold between an segment and a polygon
B2_API b2Manifold b2CollideSegmentAndPolygon( const b2Segment* segmentA, b2Transform xfA, const b2Polygon* polygonB,
											  b2Transform xfB );

/// Compute the contact manifold between a chain segment and a circle
B2_API b2Manifold b2CollideChainSegmentAndCircle( const b2ChainSegment* segmentA, b2Transform xfA, const b2Circle* circleB,
												  b2Transform xfB );

/// Compute the contact manifold between a chain segment and a capsule
B2_API b2Manifold b2CollideChainSegmentAndCapsule( const b2ChainSegment* segmentA, b2Transform xfA, const b2Capsule* capsuleB,
												   b2Transform xfB, b2DistanceCache* cache );

/// Compute the contact manifold between a chain segment and a rounded polygon
B2_API b2Manifold b2CollideChainSegmentAndPolygon( const b2ChainSegment* segmentA, b2Transform xfA, const b2Polygon* polygonB,
												   b2Transform xfB, b2DistanceCache* cache );

/**@}*/

/**
 * @defgroup tree Dynamic Tree
 * The dynamic tree is a binary AABB tree to organize and query large numbers of geometric objects
 *
 * Box2D uses the dynamic tree internally to sort collision shapes into a binary bounding volume hierarchy.
 * This data structure may have uses in games for organizing other geometry data and may be used independently
 * of Box2D rigid body simulation.
 *
 * A dynamic AABB tree broad-phase, inspired by Nathanael Presson's btDbvt.
 * A dynamic tree arranges data in a binary tree to accelerate
 * queries such as AABB queries and ray casts. Leaf nodes are proxies
 * with an AABB. These are used to hold a user collision object, such as a reference to a b2Shape.
 * Nodes are pooled and relocatable, so I use node indices rather than pointers.
 * The dynamic tree is made available for advanced users that would like to use it to organize
 * spatial game data besides rigid bodies.
 *
 * @note This is an advanced feature and normally not used by applications directly.
 * @{
 */

/// The default category bit for a tree proxy. Used for collision filtering.
#define b2_defaultCategoryBits ( 1 )

/// Convenience mask bits to use when you don't need collision filtering and just want all results.
#define b2_defaultMaskBits ( UINT64_MAX )

/// A node in the dynamic tree. This is private data placed here for performance reasons.
typedef struct b2TreeNode
{
	/// The node bounding box
	b2AABB aabb; // 16

	/// Category bits for collision filtering
	uint64_t categoryBits; // 8

	union
	{
		/// The node parent index (allocated node)
		int32_t parent;

		/// The node freelist next index (free node)
		int32_t next;
	}; // 4

	/// Child 1 index (internal node)
	int32_t child1; // 4

	union
	{
		/// Child 2 index (internal node)
		int32_t child2;

		/// User data (leaf node)
		int32_t userData;
	}; // 4

	uint16_t height; // 2
	uint16_t flags;	 // 2
} b2TreeNode;

/// The dynamic tree structure. This should be considered private data.
/// It is placed here for performance reasons.
typedef struct b2DynamicTree
{
	/// The tree nodes
	b2TreeNode* nodes;

	/// The root index
	int32_t root;

	/// The number of nodes
	int32_t nodeCount;

	/// The allocated node space
	int32_t nodeCapacity;

	/// Node free list
	int32_t freeList;

	/// Number of proxies created
	int32_t proxyCount;

	/// Leaf indices for rebuild
	int32_t* leafIndices;

	/// Leaf bounding boxes for rebuild
	b2AABB* leafBoxes;

	/// Leaf bounding box centers for rebuild
	b2Vec2* leafCenters;

	/// Bins for sorting during rebuild
	int32_t* binIndices;

	/// Allocated space for rebuilding
	int32_t rebuildCapacity;
} b2DynamicTree;

/// These are performance results returned by dynamic tree queries.
typedef struct b2TreeStats
{
	/// Number of internal nodes visited during the query
	int32_t nodeVisits;

	/// Number of leaf nodes visited during the query
	int32_t leafVisits;
} b2TreeStats;

/// Constructing the tree initializes the node pool.
B2_API b2DynamicTree b2DynamicTree_Create( void );

/// Destroy the tree, freeing the node pool.
B2_API void b2DynamicTree_Destroy( b2DynamicTree* tree );

/// Create a proxy. Provide an AABB and a userData value.
B2_API int32_t b2DynamicTree_CreateProxy( b2DynamicTree* tree, b2AABB aabb, uint64_t categoryBits, int32_t userData );

/// Destroy a proxy. This asserts if the id is invalid.
B2_API void b2DynamicTree_DestroyProxy( b2DynamicTree* tree, int32_t proxyId );

/// Move a proxy to a new AABB by removing and reinserting into the tree.
B2_API void b2DynamicTree_MoveProxy( b2DynamicTree* tree, int32_t proxyId, b2AABB aabb );

/// Enlarge a proxy and enlarge ancestors as necessary.
B2_API void b2DynamicTree_EnlargeProxy( b2DynamicTree* tree, int32_t proxyId, b2AABB aabb );

/// This function receives proxies found in the AABB query.
/// @return true if the query should continue
typedef bool b2TreeQueryCallbackFcn( int32_t proxyId, int32_t userData, void* context );

/// Query an AABB for overlapping proxies. The callback class is called for each proxy that overlaps the supplied AABB.
///	@return performance data
B2_API b2TreeStats b2DynamicTree_Query( const b2DynamicTree* tree, b2AABB aabb, uint64_t maskBits,
										b2TreeQueryCallbackFcn* callback, void* context );

/// This function receives clipped ray cast input for a proxy. The function
/// returns the new ray fraction.
/// - return a value of 0 to terminate the ray cast
/// - return a value less than input->maxFraction to clip the ray
/// - return a value of input->maxFraction to continue the ray cast without clipping
typedef float b2TreeRayCastCallbackFcn( const b2RayCastInput* input, int32_t proxyId, int32_t userData, void* context );

/// Ray cast against the proxies in the tree. This relies on the callback
/// to perform a exact ray cast in the case were the proxy contains a shape.
/// The callback also performs the any collision filtering. This has performance
/// roughly equal to k * log(n), where k is the number of collisions and n is the
/// number of proxies in the tree.
/// Bit-wise filtering using mask bits can greatly improve performance in some scenarios.
///	However, this filtering may be approximate, so the user should still apply filtering to results.
/// @param tree the dynamic tree to ray cast
/// @param input the ray cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1)
/// @param maskBits mask bit hint: `bool accept = (maskBits & node->categoryBits) != 0;`
/// @param callback a callback class that is called for each proxy that is hit by the ray
/// @param context user context that is passed to the callback
///	@return performance data
B2_API b2TreeStats b2DynamicTree_RayCast( const b2DynamicTree* tree, const b2RayCastInput* input, uint64_t maskBits,
										  b2TreeRayCastCallbackFcn* callback, void* context );

/// This function receives clipped ray cast input for a proxy. The function
/// returns the new ray fraction.
/// - return a value of 0 to terminate the ray cast
/// - return a value less than input->maxFraction to clip the ray
/// - return a value of input->maxFraction to continue the ray cast without clipping
typedef float b2TreeShapeCastCallbackFcn( const b2ShapeCastInput* input, int32_t proxyId, int32_t userData, void* context );

/// Ray cast against the proxies in the tree. This relies on the callback
/// to perform a exact ray cast in the case were the proxy contains a shape.
/// The callback also performs the any collision filtering. This has performance
/// roughly equal to k * log(n), where k is the number of collisions and n is the
/// number of proxies in the tree.
/// @param tree the dynamic tree to ray cast
/// @param input the ray cast input data. The ray extends from p1 to p1 + maxFraction * (p2 - p1).
/// @param maskBits filter bits: `bool accept = (maskBits & node->categoryBits) != 0;`
/// @param callback a callback class that is called for each proxy that is hit by the shape
/// @param context user context that is passed to the callback
///	@return performance data
B2_API b2TreeStats b2DynamicTree_ShapeCast( const b2DynamicTree* tree, const b2ShapeCastInput* input, uint64_t maskBits,
											b2TreeShapeCastCallbackFcn* callback, void* context );

/// Validate this tree. For testing.
B2_API void b2DynamicTree_Validate( const b2DynamicTree* tree );

/// Compute the height of the binary tree in O(N) time. Should not be
/// called often.
B2_API int b2DynamicTree_GetHeight( const b2DynamicTree* tree );

/// Get the maximum balance of the tree. The balance is the difference in height of the two children of a node.
B2_API int b2DynamicTree_GetMaxBalance( const b2DynamicTree* tree );

/// Get the ratio of the sum of the node areas to the root area.
B2_API float b2DynamicTree_GetAreaRatio( const b2DynamicTree* tree );

/// Build an optimal tree. Very expensive. For testing.
B2_API void b2DynamicTree_RebuildBottomUp( b2DynamicTree* tree );

/// Get the number of proxies created
B2_API int b2DynamicTree_GetProxyCount( const b2DynamicTree* tree );

/// Rebuild the tree while retaining subtrees that haven't changed. Returns the number of boxes sorted.
B2_API int b2DynamicTree_Rebuild( b2DynamicTree* tree, bool fullBuild );

/// Shift the world origin. Useful for large worlds.
/// The shift formula is: position -= newOrigin
/// @param tree the tree to shift
/// @param newOrigin the new origin with respect to the old origin
B2_API void b2DynamicTree_ShiftOrigin( b2DynamicTree* tree, b2Vec2 newOrigin );

/// Get the number of bytes used by this tree
B2_API int b2DynamicTree_GetByteCount( const b2DynamicTree* tree );

/// Get proxy user data
B2_INLINE int32_t b2DynamicTree_GetUserData( const b2DynamicTree* tree, int32_t proxyId )
{
	return tree->nodes[proxyId].userData;
}

/// Get the AABB of a proxy
B2_INLINE b2AABB b2DynamicTree_GetAABB( const b2DynamicTree* tree, int32_t proxyId )
{
	return tree->nodes[proxyId].aabb;
}

/**@}*/
