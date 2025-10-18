// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

typedef struct Camera
{
	b2Vec2 m_center;
	float m_zoom;
	float m_width;
	float m_height;
} Camera;

typedef struct Draw Draw;

#ifdef __cplusplus
extern "C"
{
#endif

Camera GetDefaultCamera( void );
void ResetView( Camera* camera );
b2Vec2 ConvertScreenToWorld( Camera* camera, b2Vec2 screenPoint );
b2Vec2 ConvertWorldToScreen( Camera* camera, b2Vec2 worldPoint );
void BuildProjectionMatrix( Camera* camera, float* m, float zBias );
b2AABB GetViewBounds( Camera* camera );


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

void DrawScreenString( Draw* draw, int x, int y, const char* string, ... );

void DrawWorldString( Draw* draw, Camera* camera, b2Vec2 p, const char* string, ... );

void DrawBounds( Draw* draw, b2AABB aabb, b2HexColor color );

void FlushDraw( Draw* draw, Camera* camera );
void DrawBackground( Draw* draw, Camera* camera  );

#ifdef __cplusplus
}
#endif
