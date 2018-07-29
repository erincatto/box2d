/*
* Copyright (c) 2006-2013 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

#include "DebugDraw.h"

#include <stdio.h>
#include <stdarg.h>

#include "Testbed/imgui/imgui.h"

DebugDraw g_debugDraw;
Camera g_camera;

static ImVec2 toImVec2(const b2Vec2& v) {
	return ImVec2(v.x, v.y);
}

static ImU32 toImColor(const b2Color& color) {
	return ImU32(color.r * 255) << 0 | ImU32(color.g * 255) << 8 | ImU32(color.b * 255) << 16 | ImU32(color.a * 255) << 24;
}

//
b2Vec2 Camera::ConvertScreenToWorld(const b2Vec2& ps)
{
    float32 w = float32(m_width);
    float32 h = float32(m_height);
	float32 u = ps.x / w;
	float32 v = (h - ps.y) / h;

	float32 ratio = w / h;
	b2Vec2 extents(ratio * 25.0f, 25.0f);
	extents *= m_zoom;

	b2Vec2 lower = m_center - extents;
	b2Vec2 upper = m_center + extents;

	b2Vec2 pw;
	pw.x = (1.0f - u) * lower.x + u * upper.x;
	pw.y = (1.0f - v) * lower.y + v * upper.y;
	return pw;
}

//
b2Vec2 Camera::ConvertWorldToScreen(const b2Vec2& pw)
{
	float32 w = float32(m_width);
	float32 h = float32(m_height);
	float32 ratio = w / h;
	b2Vec2 extents(ratio * 25.0f, 25.0f);
	extents *= m_zoom;

	b2Vec2 lower = m_center - extents;
	b2Vec2 upper = m_center + extents;

	float32 u = (pw.x - lower.x) / (upper.x - lower.x);
	float32 v = (pw.y - lower.y) / (upper.y - lower.y);

	b2Vec2 ps;
	ps.x = u * w;
	ps.y = (1.0f - v) * h;
	return ps;
}

//
void DebugDraw::DrawPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color)
{
	ImVec2 screen_vertices[vertexCount];
	for (int i = 0; i < vertexCount; ++i)
	{
		screen_vertices[i] = toImVec2(g_camera.ConvertWorldToScreen(vertices[i]));
	}

	ImGui::Begin("Overlay");
	ImDrawList* draw_list = ImGui::GetWindowDrawList();
	draw_list->AddPolyline(screen_vertices, vertexCount, toImColor(color), true, 1.0, false);
	ImGui::End();
}

//
void DebugDraw::DrawSolidPolygon(const b2Vec2* vertices, int32 vertexCount, const b2Color& color)
{
	b2Color fillColor(0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f);
	ImVec2 screen_vertices[vertexCount];
	for (int i = 0; i < vertexCount; ++i)
	{
		screen_vertices[i] = toImVec2(g_camera.ConvertWorldToScreen(vertices[i]));
	}

	ImGui::Begin("Overlay");
	ImDrawList* draw_list = ImGui::GetWindowDrawList();
	draw_list->AddConvexPolyFilled(screen_vertices, vertexCount, toImColor(fillColor), false);
	draw_list->AddPolyline(screen_vertices, vertexCount, toImColor(color), true, 1.0, false);
	ImGui::End();
}

//
void DebugDraw::DrawCircle(const b2Vec2& center, float32 radius, const b2Color& color)
{
	const int k_segments = 16;
	b2Vec2 screen_center = g_camera.ConvertWorldToScreen(center);
	b2Vec2 screen_offset = g_camera.ConvertWorldToScreen(center + b2Vec2(radius, 0.0f));
	float screen_radius = (screen_offset - screen_center).Length();

	ImGui::Begin("Overlay");
	ImDrawList* draw_list = ImGui::GetWindowDrawList();
	draw_list->AddCircle(toImVec2(screen_center), screen_radius, toImColor(color), k_segments - 1, 1.0f);
	ImGui::End();
}

//
void DebugDraw::DrawSolidCircle(const b2Vec2& center, float32 radius, const b2Vec2& axis, const b2Color& color)
{
	const int k_segments = 16;
	b2Color fillColor(0.5f * color.r, 0.5f * color.g, 0.5f * color.b, 0.5f);
	b2Vec2 screen_center = g_camera.ConvertWorldToScreen(center);
	b2Vec2 screen_offset = g_camera.ConvertWorldToScreen(center + radius * axis);
	float screen_radius = (screen_offset - screen_center).Length();

	ImGui::Begin("Overlay");
	ImDrawList* draw_list = ImGui::GetWindowDrawList();
	draw_list->AddCircleFilled(toImVec2(screen_center), screen_radius, toImColor(fillColor), k_segments - 1);
	draw_list->AddCircle(toImVec2(screen_center), screen_radius, toImColor(color), k_segments - 1, 1.0f);
	draw_list->AddLine(toImVec2(screen_center), toImVec2(screen_offset), toImColor(color));
	ImGui::End();
}

//
void DebugDraw::DrawSegment(const b2Vec2& p1, const b2Vec2& p2, const b2Color& color)
{
	b2Vec2 screen_p1 = g_camera.ConvertWorldToScreen(p1);
	b2Vec2 screen_p2 = g_camera.ConvertWorldToScreen(p2);

	ImGui::Begin("Overlay");
	ImDrawList* draw_list = ImGui::GetWindowDrawList();
	draw_list->AddLine(toImVec2(screen_p1), toImVec2(screen_p2), toImColor(color));
	ImGui::End();
}

//
void DebugDraw::DrawTransform(const b2Transform& xf)
{
	const float32 k_axisScale = 0.4f;
    b2Color red(1.0f, 0.0f, 0.0f);
    b2Color green(0.0f, 1.0f, 0.0f);

	b2Vec2 screen_p = g_camera.ConvertWorldToScreen(xf.p);
	b2Vec2 screen_px = g_camera.ConvertWorldToScreen(xf.p + k_axisScale * xf.q.GetXAxis());
	b2Vec2 screen_py = g_camera.ConvertWorldToScreen(xf.p + k_axisScale * xf.q.GetYAxis());

	ImGui::Begin("Overlay");
	ImDrawList* draw_list = ImGui::GetWindowDrawList();
	draw_list->AddLine(toImVec2(screen_p), toImVec2(screen_px), toImColor(red));
	draw_list->AddLine(toImVec2(screen_p), toImVec2(screen_py), toImColor(green));
	ImGui::End();
}

//
void DebugDraw::DrawPoint(const b2Vec2& p, float32 size, const b2Color& color)
{
	b2Vec2 hsize = b2Vec2(0.5 * size, 0.5 * size);
	b2Vec2 screen_a = g_camera.ConvertWorldToScreen(p) - hsize;
	b2Vec2 screen_b = g_camera.ConvertWorldToScreen(p) + hsize;

	ImGui::Begin("Overlay");
	ImDrawList* draw_list = ImGui::GetWindowDrawList();
	draw_list->AddRectFilled(toImVec2(screen_a), toImVec2(screen_b), toImColor(color));
	ImGui::End();
}

//
void DebugDraw::DrawString(int x, int y, const char *string, ...)
{
	va_list arg;
	va_start(arg, string);
	ImGui::Begin("Overlay");
	ImGui::SetCursorPos(ImVec2(float(x), float(y)));
	ImGui::TextColoredV(ImColor(230, 153, 153, 255), string, arg);
	ImGui::End();
	va_end(arg);
}

//
void DebugDraw::DrawString(const b2Vec2& pw, const char *string, ...)
{
	b2Vec2 ps = g_camera.ConvertWorldToScreen(pw);

	va_list arg;
	va_start(arg, string);
	ImGui::Begin("Overlay");
	ImGui::SetCursorPos(toImVec2(ps));
	ImGui::TextColoredV(ImColor(230, 153, 153, 255), string, arg);
	ImGui::End();
	va_end(arg);
}

//
void DebugDraw::DrawAABB(b2AABB* aabb, const b2Color& c)
{
	b2Vec2 screen_lower = g_camera.ConvertWorldToScreen(aabb->lowerBound);
	b2Vec2 screen_upper = g_camera.ConvertWorldToScreen(aabb->upperBound);

	ImGui::Begin("Overlay");
	ImDrawList* draw_list = ImGui::GetWindowDrawList();
	draw_list->AddRect(toImVec2(screen_lower), toImVec2(screen_upper), toImColor(c));
	ImGui::End();
}
