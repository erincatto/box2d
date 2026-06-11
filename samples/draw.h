// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

typedef struct Camera
{
	// World point the view is centered on. Double precision in large world mode.
	b2Position center;
	float zoom;
	float width;
	float height;
} Camera;

typedef struct Draw Draw;

#ifdef __cplusplus
extern "C"
{
#endif

Camera GetDefaultCamera( void );
void ResetView( Camera* camera );
b2Position ConvertScreenToWorld( Camera* camera, b2Vec2 screenPoint );
b2Vec2 ConvertWorldToScreen( Camera* camera, b2Position worldPoint );
b2AABB GetViewBounds( Camera* camera );
void FocusOnBounds( Camera* camera, b2AABB bounds );

Draw* CreateDraw( void );
void DestroyDraw( Draw* draw );

void DrawPoint( Draw* draw, b2Vec2 p, float size, b2HexColor color );
void DrawLine( Draw* draw, b2Vec2 p1, b2Vec2 p2, b2HexColor color );
void DrawCircle( Draw* draw, b2Vec2 center, float radius, b2HexColor color );
void DrawSolidCircle( Draw* draw, b2Transform transform, float radius, b2HexColor color );
void DrawSolidCapsule( Draw* draw, b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color );
void DrawPolygon( Draw* draw, const b2Vec2* vertices, int vertexCount, b2HexColor color );
void DrawSolidPolygon( Draw* draw, b2Transform transform, const b2Vec2* vertices, int vertexCount, float radius,
					   b2HexColor color );
void DrawTransform( Draw* draw, b2Transform transform, float scale );
void DrawBounds( Draw* draw, b2AABB aabb, b2HexColor color );
void DrawScreenString( Draw* draw, float x, float y, b2HexColor color, const char* string, ... );
void DrawWorldString( Draw* draw, Camera* camera, b2Position p, b2HexColor color, const char* string, ... );

void FlushDraw( Draw* draw, Camera* camera );

void DrawBackground( Draw* draw, Camera* camera );

#ifdef __cplusplus
}
#endif

#ifdef __cplusplus

// Shift an engine world coordinate into camera relative view space before drawing.
// The draw helpers below take view space float, so far from the origin the subtraction
// happens in double here and the drawn coordinate stays near screen zero. Identity in float mode.
#if defined( BOX2D_DOUBLE_PRECISION )

inline b2Vec2 CameraRelative( const Camera* camera, b2Position p )
{
	return p - camera->center;
}

inline b2Transform CameraRelative( const Camera* camera, b2WorldTransform t )
{
	return b2ToRelativeTransform( t, camera->center );
}

// A world AABB is already float, so far from the origin its corners carry the broadphase
// quantization. Shifting into view space keeps it on screen.
inline b2AABB CameraRelative( const Camera* camera, b2AABB box )
{
	b2Vec2 lower = b2PositionDelta( b2MakePosition( box.lowerBound ), camera->center );
	b2Vec2 upper = b2PositionDelta( b2MakePosition( box.upperBound ), camera->center );
	return { lower, upper };
}

#else

inline b2Vec2 CameraRelative( const Camera*, b2Vec2 p )
{
	return p;
}

inline b2Transform CameraRelative( const Camera*, b2Transform t )
{
	return t;
}

inline b2AABB CameraRelative( const Camera*, b2AABB box )
{
	return box;
}

#endif

#endif
