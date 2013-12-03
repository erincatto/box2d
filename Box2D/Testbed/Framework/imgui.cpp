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

#include <stdio.h>
#include <string.h>
//#define _USE_MATH_DEFINES
#include <math.h>
#include "imgui.h"
#include "RenderGL3.h"

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
struct GuiState
{
	GuiState()
	{
		left = false;
		leftPressed = false;
		leftReleased = false;
		mx = -1;
		my = -1;
		scroll = 0;
		active = 0;
		hot = 0;
		hotToBe = 0;
		isHot = false;
		isActive = false;
		wentActive = false;
		dragX = 0;
		dragY = 0;
		dragOrig = 0;
		widgetX = 0;
		widgetY = 0;
		widgetW = 100;
		insideCurrentScroll = false;
		areaId = 0;
		widgetId = 0;
	}

	bool left;
	bool leftPressed, leftReleased;
	int mx, my;
	int scroll;
	unsigned int active;
	unsigned int hot;
	unsigned int hotToBe;
	bool isHot;
	bool isActive;
	bool wentActive;
	int dragX, dragY;
	float dragOrig;
	int widgetX, widgetY, widgetW;
	bool insideCurrentScroll;

	unsigned int areaId;
	unsigned int widgetId;
};

static GuiState s_state;

inline bool anyActive()
{
	return s_state.active != 0;
}

inline bool isActive(unsigned int id)
{
	return s_state.active == id;
}

inline bool isHot(unsigned int id)
{
	return s_state.hot == id;
}

inline bool inRect(int x, int y, int w, int h, bool checkScroll = true)
{
	return (!checkScroll || s_state.insideCurrentScroll) && s_state.mx >= x && s_state.mx <= x + w && s_state.my >= y && s_state.my <= y + h;
}

inline void clearInput()
{
	s_state.leftPressed = false;
	s_state.leftReleased = false;
	s_state.scroll = 0;
}

inline void clearActive()
{
	s_state.active = 0;
	// mark all UI for this frame as processed
	clearInput();
}

inline void setActive(unsigned int id)
{
	s_state.active = id;
	s_state.wentActive = true;
}

inline void setHot(unsigned int id)
{
	s_state.hotToBe = id;
}


static bool buttonLogic(unsigned int id, bool over)
{
	bool res = false;
	// process down
	if (!anyActive())
	{
		if (over)
			setHot(id);
		if (isHot(id) && s_state.leftPressed)
			setActive(id);
	}

	// if button is active, then react on left up
	if (isActive(id))
	{
		s_state.isActive = true;
		if (over)
			setHot(id);
		if (s_state.leftReleased)
		{
			if (isHot(id))
				res = true;
			clearActive();
		}
	}

	if (isHot(id))
		s_state.isHot = true;

	return res;
}

static void updateInput(int mx, int my, unsigned char mbut, int scroll)
{
	bool left = (mbut & IMGUI_MBUT_LEFT) != 0;

	s_state.mx = mx;
	s_state.my = my;
	s_state.leftPressed = !s_state.left && left;
	s_state.leftReleased = s_state.left && !left;
	s_state.left = left;

	s_state.scroll = scroll;
}

void imguiBeginFrame(int mx, int my, unsigned char mbut, int scroll)
{
	updateInput(mx, my, mbut, scroll);

	s_state.hot = s_state.hotToBe;
	s_state.hotToBe = 0;

	s_state.wentActive = false;
	s_state.isActive = false;
	s_state.isHot = false;

	s_state.widgetX = 0;
	s_state.widgetY = 0;
	s_state.widgetW = 0;

	s_state.areaId = 1;
	s_state.widgetId = 1;
}

void imguiEndFrame()
{
	clearInput();
}



//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
static const int BUTTON_HEIGHT = 20;
static const int SLIDER_HEIGHT = 20;
static const int SLIDER_MARKER_WIDTH = 10;
static const int CHECK_SIZE = 8;
static const int DEFAULT_SPACING = 4;
static const int TEXT_HEIGHT = 8;
static const int SCROLL_AREA_PADDING = 6;
static const int INDENT_SIZE = 16;
static const int AREA_HEADER = 28;

static int g_scrollTop = 0;
static int g_scrollBottom = 0;
static int g_scrollRight = 0;
static int g_scrollAreaTop = 0;
static int* g_scrollVal = 0;
static int g_focusTop = 0;
static int g_focusBottom = 0;
static unsigned int g_scrollId = 0;
static bool g_insideScrollArea = false;

bool imguiBeginScrollArea(const char* name, int x, int y, int w, int h, int* scroll)
{
	s_state.areaId++;
	s_state.widgetId = 0;
	g_scrollId = (s_state.areaId << 16) | s_state.widgetId;

	s_state.widgetX = x + SCROLL_AREA_PADDING;
	s_state.widgetY = y + h - AREA_HEADER + (*scroll);
	s_state.widgetW = w - SCROLL_AREA_PADDING * 4;
	g_scrollTop = y - AREA_HEADER + h;
	g_scrollBottom = y + SCROLL_AREA_PADDING;
	g_scrollRight = x + w - SCROLL_AREA_PADDING * 3;
	g_scrollVal = scroll;

	g_scrollAreaTop = s_state.widgetY;

	g_focusTop = y - AREA_HEADER;
	g_focusBottom = y - AREA_HEADER + h;

	g_insideScrollArea = inRect(x, y, w, h, false);
	s_state.insideCurrentScroll = g_insideScrollArea;

	AddGfxCmdRoundedRect((float)x, (float)y, (float)w, (float)h, 6, SetRGBA(0, 0, 0, 192));

	AddGfxCmdText(x + AREA_HEADER / 2, y + h - AREA_HEADER / 2 - TEXT_HEIGHT / 2, TEXT_ALIGN_LEFT, name, SetRGBA(255, 255, 255, 128));

	AddGfxCmdScissor(x + SCROLL_AREA_PADDING, y + SCROLL_AREA_PADDING, w - SCROLL_AREA_PADDING * 4, h - AREA_HEADER - SCROLL_AREA_PADDING);

	return g_insideScrollArea;
}

void imguiEndScrollArea()
{
	// Disable scissoring.
	AddGfxCmdScissor(-1, -1, -1, -1);

	// Draw scroll bar
	int x = g_scrollRight + SCROLL_AREA_PADDING / 2;
	int y = g_scrollBottom;
	int w = SCROLL_AREA_PADDING * 2;
	int h = g_scrollTop - g_scrollBottom;

	int stop = g_scrollAreaTop;
	int sbot = s_state.widgetY;
	int sh = stop - sbot; // The scrollable area height.

	float barHeight = (float)h / (float)sh;

	if (barHeight < 1)
	{
		float barY = (float)(y - sbot) / (float)sh;
		if (barY < 0) barY = 0;
		if (barY > 1) barY = 1;

		// Handle scroll bar logic.
		unsigned int hid = g_scrollId;
		int hx = x;
		int hy = y + (int)(barY*h);
		int hw = w;
		int hh = (int)(barHeight*h);

		const int range = h - (hh - 1);
		bool over = inRect(hx, hy, hw, hh);
		buttonLogic(hid, over);
		if (isActive(hid))
		{
			float u = (float)(hy - y) / (float)range;
			if (s_state.wentActive)
			{
				s_state.dragY = s_state.my;
				s_state.dragOrig = u;
			}
			if (s_state.dragY != s_state.my)
			{
				u = s_state.dragOrig + (s_state.my - s_state.dragY) / (float)range;
				if (u < 0) u = 0;
				if (u > 1) u = 1;
				*g_scrollVal = (int)((1 - u) * (sh - h));
			}
		}

		// BG
		AddGfxCmdRoundedRect((float)x, (float)y, (float)w, (float)h, (float)w / 2 - 1, SetRGBA(0, 0, 0, 196));
		// Bar
		if (isActive(hid))
			AddGfxCmdRoundedRect((float)hx, (float)hy, (float)hw, (float)hh, (float)w / 2 - 1, SetRGBA(255, 196, 0, 196));
		else
			AddGfxCmdRoundedRect((float)hx, (float)hy, (float)hw, (float)hh, (float)w / 2 - 1, isHot(hid) ? SetRGBA(255, 196, 0, 96) : SetRGBA(255, 255, 255, 64));

		// Handle mouse scrolling.
		if (g_insideScrollArea) // && !anyActive())
		{
			if (s_state.scroll)
			{
				*g_scrollVal += 20 * s_state.scroll;
				if (*g_scrollVal < 0) *g_scrollVal = 0;
				if (*g_scrollVal >(sh - h)) *g_scrollVal = (sh - h);
			}
		}
	}
	s_state.insideCurrentScroll = false;
}

bool imguiButton(const char* text, bool enabled)
{
	s_state.widgetId++;
	unsigned int id = (s_state.areaId << 16) | s_state.widgetId;

	int x = s_state.widgetX;
	int y = s_state.widgetY - BUTTON_HEIGHT;
	int w = s_state.widgetW;
	int h = BUTTON_HEIGHT;
	s_state.widgetY -= BUTTON_HEIGHT + DEFAULT_SPACING;

	bool over = enabled && inRect(x, y, w, h);
	bool res = buttonLogic(id, over);

	AddGfxCmdRoundedRect((float)x, (float)y, (float)w, (float)h, (float)BUTTON_HEIGHT / 2 - 1, SetRGBA(128, 128, 128, isActive(id) ? 196 : 96));
	if (enabled)
		AddGfxCmdText(x + BUTTON_HEIGHT / 2, y + BUTTON_HEIGHT / 2 - TEXT_HEIGHT / 2, TEXT_ALIGN_LEFT, text, isHot(id) ? SetRGBA(255, 196, 0, 255) : SetRGBA(255, 255, 255, 200));
	else
		AddGfxCmdText(x + BUTTON_HEIGHT / 2, y + BUTTON_HEIGHT / 2 - TEXT_HEIGHT / 2, TEXT_ALIGN_LEFT, text, SetRGBA(128, 128, 128, 200));

	return res;
}

bool imguiItem(const char* text, bool enabled)
{
	s_state.widgetId++;
	unsigned int id = (s_state.areaId << 16) | s_state.widgetId;

	int x = s_state.widgetX;
	int y = s_state.widgetY - BUTTON_HEIGHT;
	int w = s_state.widgetW;
	int h = BUTTON_HEIGHT;
	s_state.widgetY -= BUTTON_HEIGHT + DEFAULT_SPACING;

	bool over = enabled && inRect(x, y, w, h);
	bool res = buttonLogic(id, over);

	if (isHot(id))
		AddGfxCmdRoundedRect((float)x, (float)y, (float)w, (float)h, 2.0f, SetRGBA(255, 196, 0, isActive(id) ? 196 : 96));

	if (enabled)
		AddGfxCmdText(x + BUTTON_HEIGHT / 2, y + BUTTON_HEIGHT / 2 - TEXT_HEIGHT / 2, TEXT_ALIGN_LEFT, text, SetRGBA(255, 255, 255, 200));
	else
		AddGfxCmdText(x + BUTTON_HEIGHT / 2, y + BUTTON_HEIGHT / 2 - TEXT_HEIGHT / 2, TEXT_ALIGN_LEFT, text, SetRGBA(128, 128, 128, 200));

	return res;
}

bool imguiCheck(const char* text, bool checked, bool enabled)
{
	s_state.widgetId++;
	unsigned int id = (s_state.areaId << 16) | s_state.widgetId;

	int x = s_state.widgetX;
	int y = s_state.widgetY - BUTTON_HEIGHT;
	int w = s_state.widgetW;
	int h = BUTTON_HEIGHT;
	s_state.widgetY -= BUTTON_HEIGHT + DEFAULT_SPACING;

	bool over = enabled && inRect(x, y, w, h);
	bool res = buttonLogic(id, over);

	const int cx = x + BUTTON_HEIGHT / 2 - CHECK_SIZE / 2;
	const int cy = y + BUTTON_HEIGHT / 2 - CHECK_SIZE / 2;
	AddGfxCmdRoundedRect((float)cx - 3, (float)cy - 3, (float)CHECK_SIZE + 6, (float)CHECK_SIZE + 6, 4, SetRGBA(128, 128, 128, isActive(id) ? 196 : 96));
	if (checked)
	{
		if (enabled)
			AddGfxCmdRoundedRect((float)cx, (float)cy, (float)CHECK_SIZE, (float)CHECK_SIZE, (float)CHECK_SIZE / 2 - 1, SetRGBA(255, 255, 255, isActive(id) ? 255 : 200));
		else
			AddGfxCmdRoundedRect((float)cx, (float)cy, (float)CHECK_SIZE, (float)CHECK_SIZE, (float)CHECK_SIZE / 2 - 1, SetRGBA(128, 128, 128, 200));
	}

	if (enabled)
		AddGfxCmdText(x + BUTTON_HEIGHT, y + BUTTON_HEIGHT / 2 - TEXT_HEIGHT / 2, TEXT_ALIGN_LEFT, text, isHot(id) ? SetRGBA(255, 196, 0, 255) : SetRGBA(255, 255, 255, 200));
	else
		AddGfxCmdText(x + BUTTON_HEIGHT, y + BUTTON_HEIGHT / 2 - TEXT_HEIGHT / 2, TEXT_ALIGN_LEFT, text, SetRGBA(128, 128, 128, 200));

	return res;
}

bool imguiCollapse(const char* text, const char* subtext, bool checked, bool enabled)
{
	s_state.widgetId++;
	unsigned int id = (s_state.areaId << 16) | s_state.widgetId;

	int x = s_state.widgetX;
	int y = s_state.widgetY - BUTTON_HEIGHT;
	int w = s_state.widgetW;
	int h = BUTTON_HEIGHT;
	s_state.widgetY -= BUTTON_HEIGHT; // + DEFAULT_SPACING;

	const int cx = x + BUTTON_HEIGHT / 2 - CHECK_SIZE / 2;
	const int cy = y + BUTTON_HEIGHT / 2 - CHECK_SIZE / 2;

	bool over = enabled && inRect(x, y, w, h);
	bool res = buttonLogic(id, over);

	if (checked)
		AddGfxCmdTriangle(cx, cy, CHECK_SIZE, CHECK_SIZE, 2, SetRGBA(255, 255, 255, isActive(id) ? 255 : 200));
	else
		AddGfxCmdTriangle(cx, cy, CHECK_SIZE, CHECK_SIZE, 1, SetRGBA(255, 255, 255, isActive(id) ? 255 : 200));

	if (enabled)
		AddGfxCmdText(x + BUTTON_HEIGHT, y + BUTTON_HEIGHT / 2 - TEXT_HEIGHT / 2, TEXT_ALIGN_LEFT, text, isHot(id) ? SetRGBA(255, 196, 0, 255) : SetRGBA(255, 255, 255, 200));
	else
		AddGfxCmdText(x + BUTTON_HEIGHT, y + BUTTON_HEIGHT / 2 - TEXT_HEIGHT / 2, TEXT_ALIGN_LEFT, text, SetRGBA(128, 128, 128, 200));

	if (subtext)
		AddGfxCmdText(x + w - BUTTON_HEIGHT / 2, y + BUTTON_HEIGHT / 2 - TEXT_HEIGHT / 2, TEXT_ALIGN_RIGHT, subtext, SetRGBA(255, 255, 255, 128));

	return res;
}

void imguiLabel(const char* text)
{
	int x = s_state.widgetX;
	int y = s_state.widgetY - BUTTON_HEIGHT;
	s_state.widgetY -= BUTTON_HEIGHT;
	AddGfxCmdText(x, y + BUTTON_HEIGHT / 2 - TEXT_HEIGHT / 2, TEXT_ALIGN_LEFT, text, SetRGBA(255, 255, 255, 255));
}

void imguiValue(const char* text)
{
	const int x = s_state.widgetX;
	const int y = s_state.widgetY - BUTTON_HEIGHT;
	const int w = s_state.widgetW;
	s_state.widgetY -= BUTTON_HEIGHT;

	AddGfxCmdText(x + w - BUTTON_HEIGHT / 2, y + BUTTON_HEIGHT / 2 - TEXT_HEIGHT / 2, TEXT_ALIGN_RIGHT, text, SetRGBA(255, 255, 255, 200));
}

bool imguiSlider(const char* text, float* val, float vmin, float vmax, float vinc, bool enabled)
{
	s_state.widgetId++;
	unsigned int id = (s_state.areaId << 16) | s_state.widgetId;

	int x = s_state.widgetX;
	int y = s_state.widgetY - BUTTON_HEIGHT;
	int w = s_state.widgetW;
	int h = SLIDER_HEIGHT;
	s_state.widgetY -= SLIDER_HEIGHT + DEFAULT_SPACING;

	AddGfxCmdRoundedRect((float)x, (float)y, (float)w, (float)h, 4.0f, SetRGBA(0, 0, 0, 128));

	const int range = w - SLIDER_MARKER_WIDTH;

	float u = (*val - vmin) / (vmax - vmin);
	if (u < 0) u = 0;
	if (u > 1) u = 1;
	int m = (int)(u * range);

	bool over = enabled && inRect(x + m, y, SLIDER_MARKER_WIDTH, SLIDER_HEIGHT);
	bool res = buttonLogic(id, over);
	bool valChanged = false;

	if (isActive(id))
	{
		if (s_state.wentActive)
		{
			s_state.dragX = s_state.mx;
			s_state.dragOrig = u;
		}
		if (s_state.dragX != s_state.mx)
		{
			u = s_state.dragOrig + (float)(s_state.mx - s_state.dragX) / (float)range;
			if (u < 0) u = 0;
			if (u > 1) u = 1;
			*val = vmin + u*(vmax - vmin);
			*val = floorf(*val / vinc + 0.5f)*vinc; // Snap to vinc
			m = (int)(u * range);
			valChanged = true;
		}
	}

	if (isActive(id))
		AddGfxCmdRoundedRect((float)(x + m), (float)y, (float)SLIDER_MARKER_WIDTH, (float)SLIDER_HEIGHT, 4.0f, SetRGBA(255, 255, 255, 255));
	else
		AddGfxCmdRoundedRect((float)(x + m), (float)y, (float)SLIDER_MARKER_WIDTH, (float)SLIDER_HEIGHT, 4.0f, isHot(id) ? SetRGBA(255, 196, 0, 128) : SetRGBA(255, 255, 255, 64));

	// TODO: fix this, take a look at 'nicenum'.
	int digits = (int)(ceilf(log10f(vinc)));
	char fmt[16];
	snprintf(fmt, 16, "%%.%df", digits >= 0 ? 0 : -digits);
	char msg[128];
	snprintf(msg, 128, fmt, *val);

	if (enabled)
	{
		AddGfxCmdText(x + SLIDER_HEIGHT / 2, y + SLIDER_HEIGHT / 2 - TEXT_HEIGHT / 2, TEXT_ALIGN_LEFT, text, isHot(id) ? SetRGBA(255, 196, 0, 255) : SetRGBA(255, 255, 255, 200));
		AddGfxCmdText(x + w - SLIDER_HEIGHT / 2, y + SLIDER_HEIGHT / 2 - TEXT_HEIGHT / 2, TEXT_ALIGN_RIGHT, msg, isHot(id) ? SetRGBA(255, 196, 0, 255) : SetRGBA(255, 255, 255, 200));
	}
	else
	{
		AddGfxCmdText(x + SLIDER_HEIGHT / 2, y + SLIDER_HEIGHT / 2 - TEXT_HEIGHT / 2, TEXT_ALIGN_LEFT, text, SetRGBA(128, 128, 128, 200));
		AddGfxCmdText(x + w - SLIDER_HEIGHT / 2, y + SLIDER_HEIGHT / 2 - TEXT_HEIGHT / 2, TEXT_ALIGN_RIGHT, msg, SetRGBA(128, 128, 128, 200));
	}

	return res || valChanged;
}


bool imguiSlider(const char* text, int* val, int vmin, int vmax, int vinc, bool enabled)
{
	s_state.widgetId++;
	unsigned int id = (s_state.areaId << 16) | s_state.widgetId;

	int x = s_state.widgetX;
	int y = s_state.widgetY - BUTTON_HEIGHT;
	int w = s_state.widgetW;
	int h = SLIDER_HEIGHT;
	s_state.widgetY -= SLIDER_HEIGHT + DEFAULT_SPACING;

	AddGfxCmdRoundedRect((float)x, (float)y, (float)w, (float)h, 4.0f, SetRGBA(0, 0, 0, 128));

	const int range = w - SLIDER_MARKER_WIDTH;

	float u = (*val - vmin) / float(vmax - vmin);
	if (u < 0) u = 0;
	if (u > 1) u = 1;
	int m = (int)(u * range);

	bool over = enabled && inRect(x + m, y, SLIDER_MARKER_WIDTH, SLIDER_HEIGHT);
	bool res = buttonLogic(id, over);
	bool valChanged = false;

	if (isActive(id))
	{
		if (s_state.wentActive)
		{
			s_state.dragX = s_state.mx;
			s_state.dragOrig = u;
		}
		if (s_state.dragX != s_state.mx)
		{
			u = s_state.dragOrig + (float)(s_state.mx - s_state.dragX) / (float)range;
			if (u < 0) u = 0;
			if (u > 1) u = 1;
			*val = int(vmin + u*(vmax - vmin));
			*val = int(floorf(*val / float(vinc) + 0.5f))*vinc; // Snap to vinc
			m = (int)(u * range);
			valChanged = true;
		}
	}

	if (isActive(id))
		AddGfxCmdRoundedRect((float)(x + m), (float)y, (float)SLIDER_MARKER_WIDTH, (float)SLIDER_HEIGHT, 4.0f, SetRGBA(255, 255, 255, 255));
	else
		AddGfxCmdRoundedRect((float)(x + m), (float)y, (float)SLIDER_MARKER_WIDTH, (float)SLIDER_HEIGHT, 4.0f, isHot(id) ? SetRGBA(255, 196, 0, 128) : SetRGBA(255, 255, 255, 64));

	char msg[128];
	snprintf(msg, 128, "%d", *val);

	if (enabled)
	{
		AddGfxCmdText(x + SLIDER_HEIGHT / 2, y + SLIDER_HEIGHT / 2 - TEXT_HEIGHT / 2, TEXT_ALIGN_LEFT, text, isHot(id) ? SetRGBA(255, 196, 0, 255) : SetRGBA(255, 255, 255, 200));
		AddGfxCmdText(x + w - SLIDER_HEIGHT / 2, y + SLIDER_HEIGHT / 2 - TEXT_HEIGHT / 2, TEXT_ALIGN_RIGHT, msg, isHot(id) ? SetRGBA(255, 196, 0, 255) : SetRGBA(255, 255, 255, 200));
	}
	else
	{
		AddGfxCmdText(x + SLIDER_HEIGHT / 2, y + SLIDER_HEIGHT / 2 - TEXT_HEIGHT / 2, TEXT_ALIGN_LEFT, text, SetRGBA(128, 128, 128, 200));
		AddGfxCmdText(x + w - SLIDER_HEIGHT / 2, y + SLIDER_HEIGHT / 2 - TEXT_HEIGHT / 2, TEXT_ALIGN_RIGHT, msg, SetRGBA(128, 128, 128, 200));
	}

	return res || valChanged;
}


void imguiIndent()
{
	s_state.widgetX += INDENT_SIZE;
	s_state.widgetW -= INDENT_SIZE;
}

void imguiUnindent()
{
	s_state.widgetX -= INDENT_SIZE;
	s_state.widgetW += INDENT_SIZE;
}

void imguiSeparator()
{
	s_state.widgetY -= DEFAULT_SPACING * 3;
}

void imguiSeparatorLine()
{
	int x = s_state.widgetX;
	int y = s_state.widgetY - DEFAULT_SPACING * 2;
	int w = s_state.widgetW;
	int h = 1;
	s_state.widgetY -= DEFAULT_SPACING * 4;

	AddGfxCmdRect((float)x, (float)y, (float)w, (float)h, SetRGBA(255, 255, 255, 32));
}
