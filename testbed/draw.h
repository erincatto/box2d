// MIT License

// Copyright (c) 2019 Erin Catto

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

#ifndef DRAW_H
#define DRAW_H

#define GLFW_INCLUDE_NONE
#include "glad/gl.h"
#include "GLFW/glfw3.h"

#include "box2d/box2d.h"

struct b2AABB;
struct GLRenderPoints;
struct GLRenderLines;
struct GLRenderTriangles;
struct GLFWwindow;

//
struct Camera
{
	Camera()
	{
		m_center.Set(0.0f, 20.0f);
		m_zoom = 1.0f;
		m_width = 1280;
		m_height = 800;
	}

	b2Vec2 ConvertScreenToWorld(const b2Vec2& screenPoint);
	b2Vec2 ConvertWorldToScreen(const b2Vec2& worldPoint);
	void BuildProjectionMatrix(float* m, float zBias);

	b2Vec2 m_center;
	float m_zoom;
	int32 m_width;
	int32 m_height;
};

// This class implements debug drawing callbacks that are invoked
// inside b2World::Step.
class DebugDraw : public b2Draw
{
public:
	DebugDraw();
	~DebugDraw();

	void Create();
	void Destroy();

	void DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) override;

	void DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color) override;

	void DrawCircle(const b2Vec2& center, float radius, const b2Color& color) override;

	void DrawSolidCircle(const b2Vec2& center, float radius, const b2Vec2& axis, const b2Color& color) override;

	void DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color) override;

	void DrawTransform(const b2Transform& xf) override;

	void DrawPoint(const b2Vec2& p, float size, const b2Color& color) override;

	void DrawString(int x, int y, const char* string, ...); 

	void DrawString(const b2Vec2& p, const char* string, ...);

	void DrawAABB(b2AABB* aabb, const b2Color& color);

	void Flush();

	bool m_showUI;
	GLRenderPoints* m_points;
	GLRenderLines* m_lines;
	GLRenderTriangles* m_triangles;
};

extern DebugDraw g_debugDraw;
extern Camera g_camera;
extern GLFWwindow* g_mainWindow;

#endif
