// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#pragma once

#include "box2d/types.h"

struct ImFont;

struct Camera
{
	Camera();

	void ResetView();
	b2Vec2 ConvertScreenToWorld( b2Vec2 screenPoint );
	b2Vec2 ConvertWorldToScreen( b2Vec2 worldPoint );
	void BuildProjectionMatrix( float* m, float zBias );
	b2AABB GetViewBounds();

	b2Vec2 m_center;
	float m_zoom;
	int m_width;
	int m_height;
};

// This class implements Box2D debug drawing callbacks
class Draw
{
public:
	Draw();
	~Draw();

	void Create();
	void Destroy();

	void DrawPolygon( const b2Vec2* vertices, int32_t vertexCount, b2HexColor color );
	void DrawSolidPolygon( b2Transform transform, const b2Vec2* vertices, int32_t vertexCount, float radius, b2HexColor color );

	void DrawCircle( b2Vec2 center, float radius, b2HexColor color );
	void DrawSolidCircle( b2Transform transform, b2Vec2 center, float radius, b2HexColor color );

	void DrawSolidCapsule( b2Vec2 p1, b2Vec2 p2, float radius, b2HexColor color );

	void DrawSegment( b2Vec2 p1, b2Vec2 p2, b2HexColor color );

	void DrawTransform( b2Transform transform );

	void DrawPoint( b2Vec2 p, float size, b2HexColor color );

	void DrawString( int x, int y, const char* string, ... );

	void DrawString( b2Vec2 p, const char* string, ... );

	void DrawAABB( b2AABB aabb, b2HexColor color );

	void Flush();
	void DrawBackground();

	bool m_showUI;
	struct GLBackground* m_background;
	struct GLPoints* m_points;
	struct GLLines* m_lines;
	struct GLTriangles* m_triangles;
	struct GLCircles* m_circles;
	struct GLSolidCircles* m_solidCircles;
	struct GLSolidCapsules* m_solidCapsules;
	struct GLSolidPolygons* m_solidPolygons;
	b2DebugDraw m_debugDraw;

	ImFont* m_smallFont;
	ImFont* m_regularFont;
	ImFont* m_mediumFont;
	ImFont* m_largeFont;
};

extern Draw g_draw;
extern Camera g_camera;
extern struct GLFWwindow* g_mainWindow;
