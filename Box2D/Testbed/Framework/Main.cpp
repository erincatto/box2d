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
#include "RenderGL3.h"
#include "DebugDraw.h"
#include "Test.h"

#if defined(__APPLE__)
#include <OpenGL/gl3.h>
#else
#include <glew/glew.h>
#endif

#include <glfw/glfw3.h>
#include <stdio.h>

#ifdef _MSC_VER
#define snprintf _snprintf
#endif

//
struct UIState
{
	bool showMenu;
	int scroll;
	int scrollarea1;
	bool mouseOverMenu;
	bool chooseTest;
};

//
namespace
{
	GLFWwindow* mainWindow = NULL;
	UIState ui;

	int32 testIndex = 0;
	int32 testSelection = 0;
	int32 testCount = 0;
	TestEntry* entry;
	Test* test;
	Settings settings;
	bool rightMouseDown;
	b2Vec2 lastp;
}

//
static void sCreateUI()
{
	ui.showMenu = true;
	ui.scroll = 0;
	ui.scrollarea1 = 0;
	ui.chooseTest = false;
	ui.mouseOverMenu = false;

	// Init UI
    const char* fontPath = "../Data/DroidSans.ttf";
    
	if (RenderGLInit(fontPath) == false)
	{
		fprintf(stderr, "Could not init GUI renderer.\n");
		assert(false);
		return;
	}
}

//
static void sResizeWindow(GLFWwindow*, int width, int height)
{
	g_camera.m_width = width;
	g_camera.m_height = height;
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
				g_camera.m_center.x -= 0.5f;
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
				g_camera.m_center.x += 0.5f;
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
				g_camera.m_center.y -= 0.5f;
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
				g_camera.m_center.y += 0.5f;
			}
			break;

		case GLFW_KEY_HOME:
			// Reset view
			g_camera.m_zoom = 1.0f;
			g_camera.m_center.Set(0.0f, 20.0f);
			break;

		case GLFW_KEY_Z:
			// Zoom out
			g_camera.m_zoom = b2Min(1.1f * g_camera.m_zoom, 20.0f);
			break;

		case GLFW_KEY_X:
			// Zoom in
			g_camera.m_zoom = b2Max(0.9f * g_camera.m_zoom, 0.02f);
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
	b2Vec2 ps((float32)xd, (float32)yd);

	// Use the mouse to move things around.
	if (button == GLFW_MOUSE_BUTTON_1)
	{
        //<##>
        //ps.Set(0, 0);
		b2Vec2 pw = g_camera.ConvertScreenToWorld(ps);
		if (action == GLFW_PRESS)
		{
			if (mods == GLFW_MOD_SHIFT)
			{
				test->ShiftMouseDown(pw);
			}
			else
			{
				test->MouseDown(pw);
			}
		}
		
		if (action == GLFW_RELEASE)
		{
			test->MouseUp(pw);
		}
	}
	else if (button == GLFW_MOUSE_BUTTON_2)
	{
		if (action == GLFW_PRESS)
		{	
			lastp = g_camera.ConvertScreenToWorld(ps);
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
	b2Vec2 ps((float)xd, (float)yd);

	b2Vec2 pw = g_camera.ConvertScreenToWorld(ps);
	test->MouseMove(pw);
	
	if (rightMouseDown)
	{
		b2Vec2 diff = pw - lastp;
		g_camera.m_center.x -= diff.x;
		g_camera.m_center.y -= diff.y;
		lastp = g_camera.ConvertScreenToWorld(ps);
	}
}

//
static void sScrollCallback(GLFWwindow*, double, double dy)
{
	if (ui.mouseOverMenu)
	{
		ui.scroll = -int(dy);
	}
	else
	{
		if (dy > 0)
		{
			g_camera.m_zoom /= 1.1f;
		}
		else
		{
			g_camera.m_zoom *= 1.1f;
		}
	}
}

//
static void sRestart()
{
	delete test;
	entry = g_testEntries + testIndex;
	test = entry->createFcn();
}

//
static void sSimulate()
{
	glEnable(GL_DEPTH_TEST);
	test->Step(&settings);

	test->DrawTitle(entry->name);
	glDisable(GL_DEPTH_TEST);

	if (testSelection != testIndex)
	{
		testIndex = testSelection;
		delete test;
		entry = g_testEntries + testIndex;
		test = entry->createFcn();
		g_camera.m_zoom = 1.0f;
		g_camera.m_center.Set(0.0f, 20.0f);
	}
}

//
static void sInterface()
{
	int menuWidth = 200;
	ui.mouseOverMenu = false;
	if (ui.showMenu)
	{
		bool over = imguiBeginScrollArea("Testbed Controls", g_camera.m_width - menuWidth - 10, 10, menuWidth, g_camera.m_height - 20, &ui.scrollarea1);
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
		bool over = imguiBeginScrollArea("Choose Sample", g_camera.m_width - menuWidth - testMenuWidth - 20, 10, testMenuWidth, g_camera.m_height - 20, &testScroll);
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

}

//
int main(int argc, char** argv)
{
#if defined(_WIN32)
	// Enable memory-leak reports
	_CrtSetDbgFlag(_CRTDBG_LEAK_CHECK_DF | _CrtSetDbgFlag(_CRTDBG_REPORT_FLAG));
#endif

    g_camera.m_width = 1024;
    g_camera.m_height = 640;
    
	if (glfwInit() == 0)
	{
		fprintf(stderr, "Failed to initialize GLFW\n");
		return -1;
	}

	char title[64];
	sprintf(title, "Box2D Testbed Version %d.%d.%d", b2_version.major, b2_version.minor, b2_version.revision);

#if defined(__APPLE__)
	// Not sure why, but these settings cause glewInit below to crash.
    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#endif

    mainWindow = glfwCreateWindow(g_camera.m_width, g_camera.m_height, title, NULL, NULL);
	if (mainWindow == NULL)
	{
		fprintf(stderr, "Failed to open GLFW mainWindow.\n");
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent(mainWindow);
	printf("OpenGL %s, GLSL %s\n", glGetString(GL_VERSION), glGetString(GL_SHADING_LANGUAGE_VERSION));

	glfwSetScrollCallback(mainWindow, sScrollCallback);
	glfwSetWindowSizeCallback(mainWindow, sResizeWindow);
	glfwSetKeyCallback(mainWindow, sKeyCallback);
	glfwSetMouseButtonCallback(mainWindow, sMouseButton);
	glfwSetCursorPosCallback(mainWindow, sMouseMotion);
	glfwSetScrollCallback(mainWindow, sScrollCallback);

#if defined(__APPLE__) == FALSE
	//glewExperimental = GL_TRUE;
    GLenum err = glewInit();
	if (GLEW_OK != err)
	{
		fprintf(stderr, "Error: %s\n", glewGetErrorString(err));
		exit(EXIT_FAILURE);
	}
#endif
    
	g_debugDraw.Create();

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

    double time1 = glfwGetTime();
    double frameTime = 0.0;
   
    glClearColor(0.3f, 0.3f, 0.3f, 1.f);
	
 	while (!glfwWindowShouldClose(mainWindow))
	{
 		glfwGetWindowSize(mainWindow, &g_camera.m_width, &g_camera.m_height);
		glViewport(0, 0, g_camera.m_width, g_camera.m_height);

		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		unsigned char mousebutton = 0;
		int mscroll = ui.scroll;
		ui.scroll = 0;

		double xd, yd;
		glfwGetCursorPos(mainWindow, &xd, &yd);
		int mousex = int(xd);
		int mousey = int(yd);

		mousey = g_camera.m_height - mousey;
		int leftButton = glfwGetMouseButton(mainWindow, GLFW_MOUSE_BUTTON_LEFT);
		if (leftButton == GLFW_PRESS)
			mousebutton |= IMGUI_MBUT_LEFT;

		imguiBeginFrame(mousex, mousey, mousebutton, mscroll);

		sSimulate();
		sInterface();
        
        // Measure speed
        double time2 = glfwGetTime();
        double alpha = 0.9f;
        frameTime = alpha * frameTime + (1.0 - alpha) * (time2 - time1);
        time1 = time2;

        char buffer[32];
        snprintf(buffer, 32, "%.1f ms", 1000.0 * frameTime);
        AddGfxCmdText(5, 5, TEXT_ALIGN_LEFT, buffer, WHITE);
        
		glEnable(GL_BLEND);
		glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
		glDisable(GL_DEPTH_TEST);
		RenderGLFlush(g_camera.m_width, g_camera.m_height);

		glfwSwapBuffers(mainWindow);

		glfwPollEvents();
	}

	g_debugDraw.Destroy();
	RenderGLDestroy();
	glfwTerminate();

	return 0;
}
