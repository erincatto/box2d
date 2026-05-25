// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "draw.h"

#include "imgui.h"

#include <stdarg.h>
#include <stdio.h>

extern "C" void DrawScreenString( Draw* /*draw*/, float x, float y, b2HexColor color, const char* string, ... )
{
	char buffer[256];
	va_list arg;
	va_start( arg, string );
	vsnprintf( buffer, sizeof( buffer ), string, arg );
	va_end( arg );
	buffer[255] = 0;

	// b2HexColor packs as 0xRRGGBB; matches MakeRGBA8's byte order. Force alpha to opaque.
	const ImU32 col = IM_COL32( ( color >> 16 ) & 0xFF, ( color >> 8 ) & 0xFF, color & 0xFF, 255 );

	// Old STB path treated y as the text baseline; ImGui::AddText treats y as the top of the
	// glyph. Shift up by the font ascent so existing callers work.
	ImFontBaked* baked = ImGui::GetFontBaked();
	const float ascent = baked ? baked->Ascent : 0.0f;
	ImGuiViewport* vp = ImGui::GetMainViewport();
	ImGui::GetBackgroundDrawList()->AddText( nullptr, 0.0f, ImVec2( vp->Pos.x + x, vp->Pos.y + y - ascent ), col, buffer );
}

extern "C" void DrawWorldString( Draw* draw, Camera* camera, b2Vec2 p, b2HexColor color, const char* string, ... )
{
	char buffer[256];
	va_list arg;
	va_start( arg, string );
	vsnprintf( buffer, sizeof( buffer ), string, arg );
	va_end( arg );
	buffer[255] = 0;

	b2Vec2 ps = ConvertWorldToScreen( camera, p );
	DrawScreenString( draw, ps.x, ps.y, color, "%s", buffer );
}
