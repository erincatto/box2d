// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#if defined( _MSC_VER )
#ifndef _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_WARNINGS
#endif
#define _CRTDBG_MAP_ALLOC
#endif

#define IMGUI_DISABLE_OBSOLETE_FUNCTIONS 1

#include "draw.h"
#include "sample.h"
#include "utils.h"

#include "box2d/base.h"
#include "box2d/box2d.h"
#include "box2d/math_functions.h"

// clang-format off
#include "glad/glad.h"
#include "GLFW/glfw3.h"
// clang-format on

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "implot.h"

#include "box2d/constants.h"

#include <stdio.h>
#include <stdlib.h>

#ifdef TRACY_ENABLE
#include <tracy/Tracy.hpp>
#else
#define FrameMark
#endif

#if defined( _MSC_VER )
#include <crtdbg.h>
static int MyAllocHook( int allocType, void* userData, size_t size, int blockType, long requestNumber,
						const unsigned char* filename, int lineNumber )
{
	// This hook can help find leaks
	if ( size == 143 )
	{
		size += 0;
	}

	return 1;
}
#endif

static SampleContext s_context;
static bool s_rightMouseDown = false;
static b2Vec2 s_clickPointWS = b2Vec2_zero;
static float s_framebufferScale = 1.0f;

inline bool IsPowerOfTwo( int x )
{
	return ( x != 0 ) && ( ( x & ( x - 1 ) ) == 0 );
}

void* AllocFcn( size_t size, int alignment )
{
	// Allocation must be a multiple of alignment or risk a seg fault
	// https://en.cppreference.com/w/c/memory/aligned_alloc
	assert( IsPowerOfTwo( alignment ) );
	size_t sizeAligned = ( ( size - 1 ) | ( alignment - 1 ) ) + 1;
	assert( ( sizeAligned & ( alignment - 1 ) ) == 0 );

#if defined( _MSC_VER ) || defined( __MINGW32__ ) || defined( __MINGW64__ )
	void* ptr = _aligned_malloc( sizeAligned, alignment );
#else
	void* ptr = aligned_alloc( alignment, sizeAligned );
#endif
	assert( ptr != nullptr );
	return ptr;
}

void FreeFcn( void* mem, size_t size )
{
	(void)size;

#if defined( _MSC_VER ) || defined( __MINGW32__ ) || defined( __MINGW64__ )
	_aligned_free( mem );
#else
	free( mem );
#endif
}

int AssertFcn( const char* condition, const char* fileName, int lineNumber )
{
	printf( "SAMPLE ASSERTION: %s, %s, line %d\n", condition, fileName, lineNumber );
	return 1;
}

void glfwErrorCallback( int error, const char* description )
{
	fprintf( stderr, "GLFW error occurred. Code: %d. Description: %s\n", error, description );
}

static int CompareSamples( const void* a, const void* b )
{
	SampleEntry* sa = (SampleEntry*)a;
	SampleEntry* sb = (SampleEntry*)b;

	int result = strcmp( sa->category, sb->category );
	if ( result == 0 )
	{
		result = strcmp( sa->name, sb->name );
	}

	return result;
}

static void SortSamples()
{
	qsort( g_sampleEntries, g_sampleCount, sizeof( SampleEntry ), CompareSamples );
}

static void ApplyUIStyle( void )
{
	ImGuiStyle& style = ImGui::GetStyle();

	// Metrics: containers round at 4px, controls at 3px - one deliberate
	// system instead of the stock mix. Padding gives rows room to breathe.
	style.WindowPadding = ImVec2( 10.0f, 10.0f );
	style.FramePadding = ImVec2( 8.0f, 4.0f );
	style.CellPadding = ImVec2( 6.0f, 4.0f );
	style.ItemSpacing = ImVec2( 8.0f, 7.0f );
	style.ItemInnerSpacing = ImVec2( 7.0f, 4.0f );
	style.IndentSpacing = 18.0f;
	style.ScrollbarSize = 12.0f;
	style.GrabMinSize = 10.0f;

	style.WindowBorderSize = 1.0f;
	style.FrameBorderSize = 0.0f;
	style.PopupBorderSize = 1.0f;
	style.TabBorderSize = 0.0f;
	style.SeparatorTextBorderSize = 1.0f;

	style.WindowRounding = 4.0f;
	style.ChildRounding = 4.0f;
	style.PopupRounding = 4.0f;
	style.FrameRounding = 3.0f;
	style.GrabRounding = 3.0f;
	style.ScrollbarRounding = 3.0f;
	style.TabRounding = 3.0f;

	style.WindowTitleAlign = ImVec2( 0.0f, 0.5f );

	// Palette: neutral charcoal surfaces, one steel-blue accent at three
	// brightnesses. Replaces stock ImGui's saturated cornflower blue.
	const ImVec4 accent = ImVec4( 0.28f, 0.48f, 0.66f, 1.00f );
	const ImVec4 accentHi = ImVec4( 0.38f, 0.60f, 0.80f, 1.00f );
	const ImVec4 accentLo = ImVec4( 0.22f, 0.36f, 0.50f, 1.00f );

	ImVec4* c = style.Colors;
	c[ImGuiCol_Text] = ImVec4( 0.90f, 0.91f, 0.93f, 1.00f );
	c[ImGuiCol_TextDisabled] = ImVec4( 0.49f, 0.51f, 0.55f, 1.00f );
	c[ImGuiCol_WindowBg] = ImVec4( 0.110f, 0.115f, 0.125f, 0.97f );
	c[ImGuiCol_ChildBg] = ImVec4( 0.00f, 0.00f, 0.00f, 0.00f );
	c[ImGuiCol_PopupBg] = ImVec4( 0.100f, 0.105f, 0.115f, 0.98f );
	c[ImGuiCol_Border] = ImVec4( 0.00f, 0.00f, 0.00f, 0.45f );
	c[ImGuiCol_BorderShadow] = ImVec4( 0.00f, 0.00f, 0.00f, 0.00f );
	c[ImGuiCol_FrameBg] = ImVec4( 0.18f, 0.19f, 0.21f, 1.00f );
	c[ImGuiCol_FrameBgHovered] = ImVec4( 0.24f, 0.26f, 0.29f, 1.00f );
	c[ImGuiCol_FrameBgActive] = ImVec4( 0.29f, 0.32f, 0.36f, 1.00f );
	c[ImGuiCol_TitleBg] = ImVec4( 0.090f, 0.095f, 0.105f, 1.00f );
	c[ImGuiCol_TitleBgActive] = ImVec4( 0.14f, 0.16f, 0.19f, 1.00f );
	c[ImGuiCol_TitleBgCollapsed] = ImVec4( 0.090f, 0.095f, 0.105f, 0.75f );
	c[ImGuiCol_MenuBarBg] = ImVec4( 0.13f, 0.14f, 0.16f, 1.00f );
	c[ImGuiCol_ScrollbarBg] = ImVec4( 0.06f, 0.06f, 0.07f, 0.55f );
	c[ImGuiCol_ScrollbarGrab] = ImVec4( 0.28f, 0.30f, 0.33f, 1.00f );
	c[ImGuiCol_ScrollbarGrabHovered] = ImVec4( 0.36f, 0.39f, 0.43f, 1.00f );
	c[ImGuiCol_ScrollbarGrabActive] = accent;
	c[ImGuiCol_CheckMark] = accentHi;
	c[ImGuiCol_SliderGrab] = accent;
	c[ImGuiCol_SliderGrabActive] = accentHi;
	c[ImGuiCol_Button] = ImVec4( 0.22f, 0.24f, 0.27f, 1.00f );
	c[ImGuiCol_ButtonHovered] = accentLo;
	c[ImGuiCol_ButtonActive] = accent;
	c[ImGuiCol_Header] = ImVec4( 0.19f, 0.21f, 0.24f, 1.00f );
	c[ImGuiCol_HeaderHovered] = accentLo;
	c[ImGuiCol_HeaderActive] = accent;
	c[ImGuiCol_Separator] = ImVec4( 1.00f, 1.00f, 1.00f, 0.09f );
	c[ImGuiCol_SeparatorHovered] = accentLo;
	c[ImGuiCol_SeparatorActive] = accent;
	c[ImGuiCol_ResizeGrip] = ImVec4( 1.00f, 1.00f, 1.00f, 0.06f );
	c[ImGuiCol_ResizeGripHovered] = accentLo;
	c[ImGuiCol_ResizeGripActive] = accent;
	c[ImGuiCol_Tab] = ImVec4( 0.15f, 0.16f, 0.18f, 1.00f );
	c[ImGuiCol_TabHovered] = accentLo;
	c[ImGuiCol_TabSelected] = accent;
	c[ImGuiCol_TabSelectedOverline] = accentHi;
	c[ImGuiCol_TabDimmed] = ImVec4( 0.12f, 0.13f, 0.14f, 1.00f );
	c[ImGuiCol_TabDimmedSelected] = accentLo;
	c[ImGuiCol_TextSelectedBg] = ImVec4( accent.x, accent.y, accent.z, 0.40f );
	c[ImGuiCol_DragDropTarget] = accentHi;
	c[ImGuiCol_NavCursor] = accentHi;
	c[ImGuiCol_PlotLines] = ImVec4( 0.70f, 0.72f, 0.75f, 1.00f );
	c[ImGuiCol_PlotLinesHovered] = accentHi;
	c[ImGuiCol_PlotHistogram] = accent;
	c[ImGuiCol_PlotHistogramHovered] = accentHi;
}

static void CreateUI( GLFWwindow* window, const char* glslVersion )
{
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImPlot::CreateContext();

	bool success = ImGui_ImplGlfw_InitForOpenGL( window, false );
	if ( success == false )
	{
		printf( "ImGui_ImplGlfw_InitForOpenGL failed\n" );
		assert( false );
	}

	success = ImGui_ImplOpenGL3_Init( glslVersion );
	if ( success == false )
	{
		printf( "ImGui_ImplOpenGL3_Init failed\n" );
		assert( false );
	}

	ImGuiIO& io = ImGui::GetIO();
	ApplyUIStyle();

	ImGuiStyle& style = ImGui::GetStyle();
	style.ScaleAllSizes( s_context.uiScale );
	style.FontSizeBase = floorf( 13.0f * s_context.uiScale );

	if ( s_context.uiScale == 1.0f && s_framebufferScale == 1.0f )
	{
		io.Fonts->AddFontDefaultBitmap();
	}
	else
	{
		io.Fonts->AddFontDefaultVector();
	}
}

static void DestroyUI()
{
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImPlot::DestroyContext();
	ImGui::DestroyContext();
}

static void ResizeWindowCallback( GLFWwindow*, int width, int height )
{
	s_context.camera.width = float( width );
	s_context.camera.height = float( height );
}

static void KeyCallback( GLFWwindow* window, int key, int scancode, int action, int mods )
{
	ImGui_ImplGlfw_KeyCallback( window, key, scancode, action, mods );
	if ( ImGui::GetIO().WantCaptureKeyboard )
	{
		return;
	}

	if ( action == GLFW_PRESS )
	{
		switch ( key )
		{
			case GLFW_KEY_ESCAPE:
				// Quit
				glfwSetWindowShouldClose( s_context.window, GL_TRUE );
				break;

			case GLFW_KEY_LEFT:
				// Pan left
				s_context.camera.center.x -= 0.5f;
				break;

			case GLFW_KEY_RIGHT:
				// Pan right
				s_context.camera.center.x += 0.5f;
				break;

			case GLFW_KEY_DOWN:
				s_context.camera.center.y -= 0.5f;
				break;

			case GLFW_KEY_UP:
				s_context.camera.center.y += 0.5f;
				break;

			case GLFW_KEY_HOME:
				ResetView( &s_context.camera );
				break;

			case GLFW_KEY_R:
				SelectSample( &s_context, s_context.sampleIndex, true );
				break;

			case GLFW_KEY_O:
				if ( mods == GLFW_MOD_CONTROL )
				{
					s_context.showUI = true;
					s_context.openSamplePicker = true;
				}
				else
				{
					s_context.singleStep = true;
				}
				break;

			case GLFW_KEY_SPACE:
				s_context.pause = !s_context.pause;
				break;

			case GLFW_KEY_LEFT_BRACKET:
				// Switch to previous test
				{
					int selection = s_context.sampleIndex - 1;
					if ( selection < 0 )
					{
						selection = g_sampleCount - 1;
					}
					SelectSample( &s_context, selection, false );
				}
				break;

			case GLFW_KEY_RIGHT_BRACKET:
				// Switch to next test
				{
					int selection = s_context.sampleIndex + 1;
					if ( selection == g_sampleCount )
					{
						selection = 0;
					}
					SelectSample( &s_context, selection, false );
				}
				break;

			case GLFW_KEY_TAB:
				s_context.showUI = !s_context.showUI;
				break;

			case GLFW_KEY_M:
				s_context.showMetrics = !s_context.showMetrics;
				break;

			default:
				if ( s_context.sample != nullptr )
				{
					s_context.sample->Keyboard( key );
				}
		}
	}
}

static void CharCallback( GLFWwindow* window, unsigned int c )
{
	ImGui_ImplGlfw_CharCallback( window, c );
}

static void MouseButtonCallback( GLFWwindow* window, int button, int action, int modifiers )
{
	ImGui_ImplGlfw_MouseButtonCallback( window, button, action, modifiers );

	if ( ImGui::GetIO().WantCaptureMouse )
	{
		return;
	}

	double xd, yd;
	glfwGetCursorPos( window, &xd, &yd );
	b2Vec2 ps = { float( xd ), float( yd ) };

	// Use the mouse to move things around.
	if ( button == GLFW_MOUSE_BUTTON_1 )
	{
		b2Vec2 pw = ConvertScreenToWorld( &s_context.camera, ps );
		if ( action == GLFW_PRESS )
		{
			s_context.sample->MouseDown( pw, button, modifiers );
		}

		if ( action == GLFW_RELEASE )
		{
			s_context.sample->MouseUp( pw, button );
		}
	}
	else if ( button == GLFW_MOUSE_BUTTON_2 )
	{
		if ( action == GLFW_PRESS )
		{
			s_clickPointWS = ConvertScreenToWorld( &s_context.camera, ps );
			s_rightMouseDown = true;
		}

		if ( action == GLFW_RELEASE )
		{
			s_rightMouseDown = false;
		}
	}
}

static void MouseMotionCallback( GLFWwindow* window, double xd, double yd )
{
	b2Vec2 ps = { float( xd ), float( yd ) };

	ImGui_ImplGlfw_CursorPosCallback( window, ps.x, ps.y );

	b2Vec2 pw = ConvertScreenToWorld( &s_context.camera, ps );
	s_context.sample->MouseMove( pw );

	if ( s_rightMouseDown )
	{
		b2Vec2 diff = b2Sub( pw, s_clickPointWS );
		s_context.camera.center.x -= diff.x;
		s_context.camera.center.y -= diff.y;
		s_clickPointWS = ConvertScreenToWorld( &s_context.camera, ps );
	}
}

static void ScrollCallback( GLFWwindow* window, double dx, double dy )
{
	ImGui_ImplGlfw_ScrollCallback( window, dx, dy );
	if ( ImGui::GetIO().WantCaptureMouse )
	{
		return;
	}

	double xd, yd;
	glfwGetCursorPos( window, &xd, &yd );
	b2Vec2 ps = { (float)xd, (float)yd };
	b2Vec2 pw1 = ConvertScreenToWorld( &s_context.camera, ps );

	if ( dy > 0 )
	{
		s_context.camera.zoom /= 1.1f;
	}
	else
	{
		s_context.camera.zoom *= 1.1f;
	}

	b2Vec2 pw2 = ConvertScreenToWorld( &s_context.camera, ps );
	s_context.camera.center -= pw2 - pw1;
}

int main( int argc, char** argv )
{
#if defined( _MSC_VER )
	// Enable memory-leak reports
	//_CrtSetBreakAlloc( 1418 );
	_CrtSetReportMode( _CRT_WARN, _CRTDBG_MODE_DEBUG | _CRTDBG_MODE_FILE );
	_CrtSetReportFile( _CRT_WARN, _CRTDBG_FILE_STDOUT );
	//_CrtSetAllocHook(MyAllocHook);
#endif

#ifdef TRACY_ENABLE
	tracy::StartupProfiler();
#endif

	// Install memory hooks
	b2SetAllocator( AllocFcn, FreeFcn );
	b2SetAssertFcn( AssertFcn );

	char buffer[128];

	s_context.Load();
	s_context.workerCount = b2MinInt( 8, GetNumberOfCores() / 2 );

	// A recording path on the command line opens straight into the replay viewer.
	// Dragging a file onto the exe arrives here as argv[1] too.
	if ( argc > 1 && g_replayIndex >= 0 )
	{
		snprintf( s_context.replayFile, sizeof( s_context.replayFile ), "%s", argv[1] );
		s_context.sampleIndex = g_replayIndex;
	}

	SortSamples();

	glfwSetErrorCallback( glfwErrorCallback );

	if ( glfwInit() == 0 )
	{
		fprintf( stderr, "Failed to initialize GLFW\n" );
		return -1;
	}

#if __APPLE__
	const char* glslVersion = "#version 150";
#else
	const char* glslVersion = nullptr;
#endif

	glfwWindowHint( GLFW_CONTEXT_VERSION_MAJOR, 3 );
	glfwWindowHint( GLFW_CONTEXT_VERSION_MINOR, 3 );
	glfwWindowHint( GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE );
	glfwWindowHint( GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE );

	// MSAA
	glfwWindowHint( GLFW_SAMPLES, 4 );

	b2Version version = b2GetVersion();
	snprintf( buffer, 128, "Box2D Version %d.%d.%d", version.major, version.minor, version.revision );

	if ( GLFWmonitor* primaryMonitor = glfwGetPrimaryMonitor() )
	{
		float contentScale = 1.0f;
		glfwGetMonitorContentScale( primaryMonitor, &contentScale, &contentScale );

#ifdef __APPLE__
		s_context.uiScale = 1.0f;
		s_framebufferScale = contentScale;
#else
		s_context.uiScale = contentScale;
		s_framebufferScale = 1.0f;
#endif
	}

	bool fullscreen = false;
	if ( fullscreen )
	{
		s_context.window = glfwCreateWindow( 1920, 1080, buffer, glfwGetPrimaryMonitor(), nullptr );
	}
	else
	{
		s_context.window =
			glfwCreateWindow( int( s_context.camera.width ), int( s_context.camera.height ), buffer, nullptr, nullptr );
	}

	if ( s_context.window == nullptr )
	{
		fprintf( stderr, "Failed to open GLFW window.\n" );
		glfwTerminate();
		return -1;
	}

	glfwMakeContextCurrent( s_context.window );

	// Load OpenGL functions using glad
	if ( !gladLoadGL() )
	{
		fprintf( stderr, "Failed to initialize glad\n" );
		glfwTerminate();
		return -1;
	}

	{
		const char* glVersionString = (const char*)glGetString( GL_VERSION );
		const char* glslVersionString = (const char*)glGetString( GL_SHADING_LANGUAGE_VERSION );
		printf( "OpenGL %s, GLSL %s\n", glVersionString, glslVersionString );
	}

	glfwSetWindowSizeCallback( s_context.window, ResizeWindowCallback );
	glfwSetKeyCallback( s_context.window, KeyCallback );
	glfwSetCharCallback( s_context.window, CharCallback );
	glfwSetMouseButtonCallback( s_context.window, MouseButtonCallback );
	glfwSetCursorPosCallback( s_context.window, MouseMotionCallback );
	glfwSetScrollCallback( s_context.window, ScrollCallback );

	CreateUI( s_context.window, glslVersion );
	s_context.draw = CreateDraw();

	s_context.sampleIndex = b2ClampInt( s_context.sampleIndex, 0, g_sampleCount - 1 );

	glClearColor( 0.2f, 0.2f, 0.2f, 1.0f );

	float frameTime = 0.0;

	while ( !glfwWindowShouldClose( s_context.window ) )
	{
		double time1 = glfwGetTime();

		if ( glfwGetKey( s_context.window, GLFW_KEY_Z ) == GLFW_PRESS )
		{
			// Zoom out
			s_context.camera.zoom = b2MinFloat( 1.005f * s_context.camera.zoom, 100.0f );
		}
		else if ( glfwGetKey( s_context.window, GLFW_KEY_X ) == GLFW_PRESS )
		{
			// Zoom in
			s_context.camera.zoom = b2MaxFloat( 0.995f * s_context.camera.zoom, 0.5f );
		}

		int width, height;
		glfwGetWindowSize( s_context.window, &width, &height );
		s_context.camera.width = width;
		s_context.camera.height = height;

		int bufferWidth, bufferHeight;
		glfwGetFramebufferSize( s_context.window, &bufferWidth, &bufferHeight );
		glViewport( 0, 0, bufferWidth, bufferHeight );

		glClear( GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT );

		// DrawBackground( s_context.draw, &s_context.camera );

		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();

		ImGuiIO& io = ImGui::GetIO();
		io.DisplaySize.x = s_context.camera.width;
		io.DisplaySize.y = s_context.camera.height;

		// These can be zero if the window is minimized
		if ( s_context.camera.width > 0.0f && s_context.camera.height > 0.0f )
		{
			// Framebuffer/window ratio: 1 on Windows/Linux, 2 on a Retina display. Drives
			// both UI magnification and font rasterizer density.
			io.DisplayFramebufferScale.x = bufferWidth / s_context.camera.width;
			io.DisplayFramebufferScale.y = bufferHeight / s_context.camera.height;
		}

		ImGui::NewFrame();

		if ( s_context.sample == nullptr )
		{
			// delayed creation because imgui doesn't create fonts until NewFrame() is called
			if ( g_sampleEntries[s_context.sampleIndex].capacityFcn != nullptr )
			{
				s_context.capacity = g_sampleEntries[s_context.sampleIndex].capacityFcn();
			}
			else
			{
				s_context.capacity = b2DefaultWorldDef().capacity;
			}
			s_context.sample = g_sampleEntries[s_context.sampleIndex].createFcn( &s_context );
		}

		s_context.sample->ResetText();

		if ( s_context.showUI == false )
		{
			// Minimal hud
			s_context.sample->DrawHud( frameTime );
		}

		s_context.sample->Step();

		FlushDraw( s_context.draw, &s_context.camera );

		if ( s_context.showUI == true )
		{
			DrawUI( &s_context, frameTime );
		}

		// ImGui::ShowDemoWindow();

		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData( ImGui::GetDrawData() );

		glfwSwapBuffers( s_context.window );

		// For the Tracy profiler
		FrameMark;

		glfwPollEvents();

		// Limit frame rate to 60Hz
		double time2 = glfwGetTime();
		double targetTime = time1 + 1.0 / 60.0;
		while ( time2 < targetTime )
		{
			b2Yield();
			time2 = glfwGetTime();
		}

		frameTime = float( time2 - time1 );
	}

	delete s_context.sample;

	DestroyDraw( s_context.draw );

	DestroyUI();
	glfwTerminate();

	s_context.Save();

#ifdef TRACY_ENABLE
	tracy::ShutdownProfiler();
#endif

#if defined( _MSC_VER )
	_CrtDumpMemoryLeaks();
#endif

	return 0;
}
