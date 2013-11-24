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

#include "imgui.h"
#include "imguiRenderGL3.h"
#include "Render.h"
#include "Test.h"

#include <glew/glew.h>
#include <glfw/glfw3.h>
#include <stdio.h>

//
struct UIState
{
	bool showMenu;
	double scroll;
	int scrollarea1;
	bool mouseOverMenu;
	bool chooseTest;
};

//
namespace
{
	GLFWwindow* mainWindow = NULL;
	int windowWidth = 1280, windowHeight = 800;
	UIState ui;

	int32 testIndex = 0;
	int32 testSelection = 0;
	int32 testCount = 0;
	TestEntry* entry;
	Test* test;
	Settings settings;
	int32 framePeriod = 16;
	float32 viewZoom = 1.0f;
	bool rightMouseDown;
	b2Vec2 lastp;
}

//
static void sCreateUI()
{
	ui.showMenu = true;
	ui.scroll = 0.0;
	ui.scrollarea1 = 0;
	ui.chooseTest = false;
	ui.mouseOverMenu = false;

	// Init UI
	if (!imguiRenderGLInit("DroidSans.ttf"))
	{
		fprintf(stderr, "Could not init GUI renderer.\n");
		assert(false);
		return;
	}
}

//
static void sResizeWindow(GLFWwindow*, int width, int height)
{
	windowWidth = width;
	windowHeight = height;
}

//
static b2Vec2 sConvertScreenToWorld(int32 x, int32 y)
{
	float32 u = x / float32(windowWidth);
	float32 v = (windowHeight - y) / float32(windowHeight);

	float32 ratio = float32(windowWidth) / float32(windowHeight);
	b2Vec2 extents(ratio * 25.0f, 25.0f);
	extents *= viewZoom;

	b2Vec2 lower = settings.viewCenter - extents;
	b2Vec2 upper = settings.viewCenter + extents;

	b2Vec2 p;
	p.x = (1.0f - u) * lower.x + u * upper.x;
	p.y = (1.0f - v) * lower.y + v * upper.y;
	return p;
}

//
static void sKeyCallback(GLFWwindow*, int key, int scancode, int action, int mods)
{
	if (action == GLFW_PRESS)
	{
		switch (key)
		{
		case GLFW_KEY_ESCAPE:
			// Quit
			glfwSetWindowShouldClose(mainWindow, GL_TRUE);
			break;

		case GLFW_KEY_LEFT:
			// Pan left
			if (mods == GLFW_MOD_CONTROL)
			{
				b2Vec2 newOrigin(2.0f, 0.0f);
				test->ShiftOrigin(newOrigin);
			}
			else
			{
				settings.viewCenter.x -= 0.5f;
				sResizeWindow(mainWindow, windowWidth, windowHeight);
			}
			break;

		case GLFW_KEY_RIGHT:
			// Pan right
			if (mods == GLFW_MOD_CONTROL)
			{
				b2Vec2 newOrigin(-2.0f, 0.0f);
				test->ShiftOrigin(newOrigin);
			}
			else
			{
				settings.viewCenter.x += 0.5f;
				sResizeWindow(mainWindow, windowWidth, windowHeight);
			}
			break;

		case GLFW_KEY_DOWN:
			// Pan down
			if (mods == GLFW_MOD_CONTROL)
			{
				b2Vec2 newOrigin(0.0f, 2.0f);
				test->ShiftOrigin(newOrigin);
			}
			else
			{
				settings.viewCenter.y -= 0.5f;
				sResizeWindow(mainWindow, windowWidth, windowHeight);
			}
			break;

		case GLFW_KEY_UP:
			// Pan up
			if (mods == GLFW_MOD_CONTROL)
			{
				b2Vec2 newOrigin(0.0f, -2.0f);
				test->ShiftOrigin(newOrigin);
			}
			else
			{
				settings.viewCenter.y += 0.5f;
				sResizeWindow(mainWindow, windowWidth, windowHeight);
			}
			break;

		case GLFW_KEY_HOME:
			// Reset view
			viewZoom = 1.0f;
			settings.viewCenter.Set(0.0f, 20.0f);
			sResizeWindow(mainWindow, windowWidth, windowHeight);
			break;

		case GLFW_KEY_Z:
			// Zoom out
			viewZoom = b2Min(1.1f * viewZoom, 20.0f);
			sResizeWindow(mainWindow, windowWidth, windowHeight);
			break;

		case GLFW_KEY_X:
			// Zoom in
			viewZoom = b2Max(0.9f * viewZoom, 0.02f);
			sResizeWindow(mainWindow, windowWidth, windowHeight);
			break;

		case GLFW_KEY_R:
			// Reset test
			delete test;
			test = entry->createFcn();
			break;

		case GLFW_KEY_SPACE:
			// Launch a bomb.
			if (test)
			{
				test->LaunchBomb();
			}
			break;

		case GLFW_KEY_P:
			// Pause
			settings.pause = !settings.pause;
			break;

		case GLFW_KEY_LEFT_BRACKET:
			// Switch to previous test
			--testSelection;
			if (testSelection < 0)
			{
				testSelection = testCount - 1;
			}
			break;

		case GLFW_KEY_RIGHT_BRACKET:
			// Switch to next test
			++testSelection;
			if (testSelection == testCount)
			{
				testSelection = 0;
			}
			break;

		case GLFW_KEY_TAB:
			ui.showMenu = !ui.showMenu;

		default:
			if (test)
			{
				test->Keyboard(key);
			}
		}
	}
	else if (action == GLFW_RELEASE)
	{
		test->KeyboardUp(key);
	}
	// else GLFW_REPEAT
}

//
static void sMouseButton(GLFWwindow*, int32 button, int32 action, int32 mods)
{
	double xd, yd;
	glfwGetCursorPos(mainWindow, &xd, &yd);
	int32 x = static_cast<int32>(floor(xd));
	int32 y = static_cast<int32>(floor(yd));

	// Use the mouse to move things around.
	if (button == GLFW_MOUSE_BUTTON_1)
	{
		b2Vec2 p = sConvertScreenToWorld(x, y);
		if (action == GLFW_PRESS)
		{
			if (mods == GLFW_MOD_SHIFT)
			{
				test->ShiftMouseDown(p);
			}
			else
			{
				test->MouseDown(p);
			}
		}
		
		if (action == GLFW_RELEASE)
		{
			test->MouseUp(p);
		}
	}
	else if (button == GLFW_MOUSE_BUTTON_2)
	{
		if (action == GLFW_PRESS)
		{	
			lastp = sConvertScreenToWorld(x, y);
			rightMouseDown = true;
		}

		if (action == GLFW_RELEASE)
		{
			rightMouseDown = false;
		}
	}
}

//
static void sMouseMotion(GLFWwindow*, double xd, double yd)
{
	int32 x = static_cast<int32>(floor(xd));
	int32 y = static_cast<int32>(floor(yd));

	b2Vec2 p = sConvertScreenToWorld(x, y);
	test->MouseMove(p);
	
	if (rightMouseDown)
	{
		b2Vec2 diff = p - lastp;
		settings.viewCenter.x -= diff.x;
		settings.viewCenter.y -= diff.y;
		sResizeWindow(mainWindow, windowWidth, windowHeight);
		lastp = sConvertScreenToWorld(x, y);
	}
}

//
static void sScrollCallback(GLFWwindow*, double, double dy)
{
	if (ui.mouseOverMenu)
	{
		ui.scroll = -dy;
	}
	else
	{
		if (dy > 0)
		{
			viewZoom /= 1.1f;
		}
		else
		{
			viewZoom *= 1.1f;
		}
	}
}

//
static void sRestart()
{
	delete test;
	entry = g_testEntries + testIndex;
	test = entry->createFcn();
	sResizeWindow(mainWindow, windowWidth, windowHeight);
}

//
static void sExit()
{
	// Quit
	glfwSetWindowShouldClose(mainWindow, GL_TRUE);
}

//
static void sSingleStep()
{
	settings.pause = 1;
	settings.singleStep = 1;
}

//
static void sSimulate()
{
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	float32 ratio = float32(windowWidth) / float32(windowHeight);

	b2Vec2 extents(ratio * 25.0f, 25.0f);
	extents *= viewZoom;

	b2Vec2 lower = settings.viewCenter - extents;
	b2Vec2 upper = settings.viewCenter + extents;

	// L/R/B/T
	gluOrtho2D(lower.x, upper.x, lower.y, upper.y);

	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();

	test->Step(&settings);

	//test->DrawTitle(entry->name);

	if (testSelection != testIndex)
	{
		testIndex = testSelection;
		delete test;
		entry = g_testEntries + testIndex;
		test = entry->createFcn();
		viewZoom = 1.0f;
		settings.viewCenter.Set(0.0f, 20.0f);
	}
}

//
static void sInterface()
{
	glEnable(GL_BLEND);
	glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
	glDisable(GL_DEPTH_TEST);

	unsigned char mousebutton = 0;
	int mscroll = int(ui.scroll);
	ui.scroll = 0;
	int mousex; int mousey;

	{
		double xd, yd;
		glfwGetCursorPos(mainWindow, &xd, &yd);
		mousex = int(xd);
		mousey = int(yd);
	}

	mousey = windowHeight - mousey;
	int leftButton = glfwGetMouseButton(mainWindow, GLFW_MOUSE_BUTTON_LEFT);
	int toggle = 0;
	if (leftButton == GLFW_PRESS)
		mousebutton |= IMGUI_MBUT_LEFT;

	int menuWidth = 200;

	ui.mouseOverMenu = false;

	imguiBeginFrame(mousex, mousey, mousebutton, mscroll);

	if (ui.showMenu)
	{
		bool over = imguiBeginScrollArea("Testbed Controls", windowWidth - menuWidth - 10, 10, menuWidth, windowHeight - 20, &ui.scrollarea1);
		if (over) ui.mouseOverMenu = true;

		imguiSeparatorLine();

		imguiLabel("Test");
		if (imguiButton(entry->name, true))
		{
			ui.chooseTest = !ui.chooseTest;
		}

		imguiSeparatorLine();

		imguiSlider("Vel Iters", &settings.velocityIterations, 0, 50, 1, true);
		imguiSlider("Pos Iters", &settings.positionIterations, 0, 50, 1, true);
		imguiSlider("Hertz", &settings.hz, 5.0f, 120.0f, 5.0f, true);

		if (imguiCheck("Sleep", settings.enableSleep, true))
			settings.enableSleep = !settings.enableSleep;
		if (imguiCheck("Warm Starting", settings.enableWarmStarting, true))
			settings.enableWarmStarting = !settings.enableWarmStarting;
		if (imguiCheck("Time of Impact", settings.enableContinuous, true))
			settings.enableContinuous = !settings.enableContinuous;
		if (imguiCheck("Sub-Stepping", settings.enableSubStepping, true))
			settings.enableSubStepping = !settings.enableSubStepping;

		imguiSeparatorLine();

		if (imguiCheck("Shapes", settings.drawShapes, true))
			settings.drawShapes = !settings.drawShapes;
		if (imguiCheck("Joints", settings.drawJoints, true))
			settings.drawJoints = !settings.drawJoints;
		if (imguiCheck("AABBs", settings.drawAABBs, true))
			settings.drawAABBs = !settings.drawAABBs;
		if (imguiCheck("Contact Points", settings.drawContactPoints, true))
			settings.drawContactPoints = !settings.drawContactPoints;
		if (imguiCheck("Contact Normals", settings.drawContactNormals, true))
			settings.drawContactNormals = !settings.drawContactNormals;
		if (imguiCheck("Contact Impulses", settings.drawContactImpulse, true))
			settings.drawContactImpulse = !settings.drawContactImpulse;
		if (imguiCheck("Friction Impulses", settings.drawFrictionImpulse, true))
			settings.drawFrictionImpulse = !settings.drawFrictionImpulse;
		if (imguiCheck("Center of Masses", settings.drawCOMs, true))
			settings.drawCOMs = !settings.drawCOMs;
		if (imguiCheck("Statistics", settings.drawStats, true))
			settings.drawStats = !settings.drawStats;
		if (imguiCheck("Profile", settings.drawProfile, true))
			settings.drawProfile = !settings.drawProfile;

		if (imguiButton("Pause", true))
			settings.pause = !settings.pause;

		if (imguiButton("Single Step", true))
			settings.singleStep = !settings.singleStep;

		if (imguiButton("Restart", true))
			sRestart();

		if (imguiButton("Quit", true))
			glfwSetWindowShouldClose(mainWindow, GL_TRUE);

		imguiEndScrollArea();
	}

	int testMenuWidth = 200;
	if (ui.chooseTest)
	{
		static int testScroll = 0;
		bool over = imguiBeginScrollArea("Choose Sample", windowWidth - menuWidth - testMenuWidth - 20, 10, testMenuWidth, windowHeight - 20, &testScroll);
		if (over) ui.mouseOverMenu = true;

		for (int i = 0; i < testCount; ++i)
		{
			if (imguiItem(g_testEntries[i].name, true))
			{
				delete test;
				entry = g_testEntries + i;
				test = entry->createFcn();
				ui.chooseTest = false;
			}
		}

		imguiEndScrollArea();
	}

	imguiEndFrame();
	imguiRenderGLDraw(windowWidth, windowHeight);
}

//
int main(int argc, char** argv)
{
	if (glfwInit() == 0)
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return -1;
	}

	char title[64];
	sprintf(title, "Box2D Testbed Version %d.%d.%d", b2_version.major, b2_version.minor, b2_version.revision);
	mainWindow = glfwCreateWindow(windowWidth, windowHeight, title, NULL, NULL);
	if (mainWindow == NULL)
	{
		fprintf(stderr, "Failed to open GLFW mainWindow.\n");
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(mainWindow);
	glfwSetScrollCallback(mainWindow, sScrollCallback);
	glfwSetWindowSizeCallback(mainWindow, sResizeWindow);
	glfwSetKeyCallback(mainWindow, sKeyCallback);
	glfwSetMouseButtonCallback(mainWindow, sMouseButton);
	glfwSetCursorPosCallback(mainWindow, sMouseMotion);
	glfwSetScrollCallback(mainWindow, sScrollCallback);

	GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
		exit(EXIT_FAILURE);
	}

	sCreateUI();

	testCount = 0;
	while (g_testEntries[testCount].createFcn != NULL)
	{
		++testCount;
	}

	testIndex = b2Clamp(testIndex, 0, testCount - 1);
	testSelection = testIndex;

	entry = g_testEntries + testIndex;
	test = entry->createFcn();

	// Control the frame rate. One draw per monitor refresh.
	glfwSwapInterval(1);

	glClearColor(0.3f, 0.3f, 0.3f, 1.f);
	// glfw scrolling
	int glfwscroll = 0;
	while (!glfwWindowShouldClose(mainWindow))
	{
		glfwGetWindowSize(mainWindow, &windowWidth, &windowHeight);
		glViewport(0, 0, windowWidth, windowHeight);

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		sSimulate();
		sInterface();

		glfwSwapBuffers(mainWindow);

		glfwPollEvents();
	}

	imguiRenderGLDestroy();
	glfwTerminate();

	return 0;
}
