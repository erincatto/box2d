//
// Copyright (c) 2009-2010 Mikko Mononen memon@inside.org
//
// This software is provided 'as-is', without any express or implied
// warranty.  In no event will the authors be held liable for any damages
// arising from the use of this software.
// Permission is granted to anyone to use this software for any purpose,
// including commercial applications, and to alter it and redistribute it
// freely, subject to the following restrictions:
// 1. The origin of this software must not be misrepresented; you must not
//    claim that you wrote the original software. If you use this software
//    in a product, an acknowledgment in the product documentation would be
//    appreciated but is not required.
// 2. Altered source versions must be plainly marked as such, and must not be
//    misrepresented as being the original software.
// 3. This notice may not be removed or altered from any source distribution.
//

// Source altered and distributed from https://github.com/AdrienHerubel/imgui

#pragma once

#include <Box2D/Common/b2Math.h>

#define SILVER (220 | (220 << 8) | (220 << 16) | (255 << 24))
#define WHITE (255 | (255 << 8) | (255 << 16) | (255 << 24))
#define RED (255 | (0 << 8) | (0 << 16) | (255 << 24))
#define GREEN (0 | (255 << 8) | (0 << 16) | (255 << 24))
#define BLUE (0 | (0 << 8) | (255 << 16) | (255 << 24))

enum TextAlign
{
	TEXT_ALIGN_LEFT,
	TEXT_ALIGN_CENTER,
	TEXT_ALIGN_RIGHT,
};

inline unsigned int SetRGBA(unsigned char r, unsigned char g, unsigned char b, unsigned char a)
{
	return (r) | (g << 8) | (b << 16) | (a << 24);
}

bool RenderGLInit(const char* fontpath);
void RenderGLDestroy();
void RenderGLFlush(int width, int height);

void AddGfxCmdScissor(int x, int y, int w, int h);
void AddGfxCmdRect(float x, float y, float w, float h, unsigned int color);
void AddGfxCmdRoundedRect(float x, float y, float w, float h, float r, unsigned int color);
void AddGfxCmdLine(float x0, float y0, float x1, float y1, float r, unsigned int color);
void AddGfxCmdTriangle(int x, int y, int w, int h, int flags, unsigned int color);
void AddGfxCmdText(float x, float y, TextAlign align, const char* text, unsigned int color);
void AddGfxCmdText(int x, int y, TextAlign align, const char* text, unsigned int color);
