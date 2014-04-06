
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

#ifndef IMGUI_H
#define IMGUI_H

enum imguiMouseButton
{
	IMGUI_MBUT_LEFT = 0x01,
	IMGUI_MBUT_RIGHT = 0x02,
};

void imguiBeginFrame(int mx, int my, unsigned char mbut, int scroll);
void imguiEndFrame();

bool imguiBeginScrollArea(const char* name, int x, int y, int w, int h, int* scroll);
void imguiEndScrollArea();

void imguiIndent();
void imguiUnindent();
void imguiSeparator();
void imguiSeparatorLine();

bool imguiButton(const char* text, bool enabled);
bool imguiItem(const char* text, bool enabled);
bool imguiCheck(const char* text, bool checked, bool enabled);
bool imguiCollapse(const char* text, const char* subtext, bool checked, bool enabled);
void imguiLabel(const char* text);
void imguiValue(const char* text);
bool imguiSlider(const char* text, float* val, float vmin, float vmax, float vinc, bool enabled);
bool imguiSlider(const char* text, int* val, int vmin, int vmax, int vinc, bool enabled);

#endif // IMGUI_H
