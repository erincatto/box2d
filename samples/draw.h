// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

typedef struct Camera
{
	// World point the view is centered on. Double precision in large world mode.
	b2Pos center;
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
b2Pos ConvertScreenToWorld( Camera* camera, b2Vec2 screenPoint );
b2Vec2 ConvertWorldToScreen( Camera* camera, b2Pos worldPoint );
b2Vec2 ConvertViewToScreen( Camera* camera, b2Vec2 viewPoint );
b2AABB GetViewBounds( Camera* camera );
void FocusOnBounds( Camera* camera, b2AABB bounds );

Draw* CreateDraw( void );
void DestroyDraw( Draw* draw );

// These take camera relative view space, float and near zero. World coordinates reach this frame
// two ways: the engine subtracts b2DebugDraw::origin for its own draws, host code uses the
// DrawWorld helpers which subtract Draw::origin. Both are identity in float mode.
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

// World space draws. Set Draw::origin to the camera center each frame so far from the origin the
// shift happens in double before reaching the float helpers above.
void SetDrawOrigin( Draw* draw, b2Pos origin );
void DrawWorldPoint( Draw* draw, b2Pos p, float size, b2HexColor color );
void DrawWorldLine( Draw* draw, b2Pos p1, b2Pos p2, b2HexColor color );
void DrawWorldCircle( Draw* draw, b2Pos center, float radius, b2HexColor color );
void DrawWorldCapsule( Draw* draw, b2Pos p1, b2Pos p2, float radius, b2HexColor color );
void DrawWorldPolygon( Draw* draw, b2WorldTransform transform, const b2Vec2* vertices, int vertexCount, b2HexColor color );
void DrawWorldSolidCircle( Draw* draw, b2WorldTransform transform, float radius, b2HexColor color );
void DrawWorldSolidPolygon( Draw* draw, b2WorldTransform transform, const b2Vec2* vertices, int vertexCount, float radius,
						   b2HexColor color );
void DrawWorldTransform( Draw* draw, b2WorldTransform t, float scale );
void DrawWorldBounds( Draw* draw, b2AABB aabb, b2HexColor color );

void DrawScreenString( Draw* draw, float x, float y, b2HexColor color, const char* string, ... );
void DrawWorldString( Draw* draw, Camera* camera, b2Pos p, b2HexColor color, const char* string, ... );

void FlushDraw( Draw* draw, Camera* camera );

void DrawBackground( Draw* draw, Camera* camera );

#ifdef __cplusplus
}
#endif
