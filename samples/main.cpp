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

void* AllocFcn( unsigned int size, int alignment )
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

void FreeFcn( void* mem, unsigned int size )
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

	ImGui::GetStyle().ScaleAllSizes( s_context.uiScale );

	const char* fontPath = "samples/data/droid_sans.ttf";
	FILE* file = fopen( fontPath, "rb" );

	if ( file != nullptr )
	{
		ImFontConfig fontConfig;
		fontConfig.RasterizerMultiply = s_context.uiScale * s_framebufferScale;

		float regularSize = floorf( 13.0f * s_context.uiScale );
		float mediumSize = floorf( 40.0f * s_context.uiScale );
		float largeSize = floorf( 64.0f * s_context.uiScale );

		ImGuiIO& io = ImGui::GetIO();
		s_context.regularFont = io.Fonts->AddFontFromFileTTF( fontPath, regularSize, &fontConfig );
		s_context.mediumFont = io.Fonts->AddFontFromFileTTF( fontPath, mediumSize, &fontConfig );
		s_context.largeFont = io.Fonts->AddFontFromFileTTF( fontPath, largeSize, &fontConfig );

		ImGui::GetIO().FontDefault = s_context.regularFont;
	}
	else
	{
		printf( "\n\nERROR: the Box2D samples working directory must be the top level Box2D directory (same as README.md)\n\n" );
		exit( EXIT_FAILURE );
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
				if ( mods == GLFW_MOD_CONTROL )
				{
					b2Vec2 newOrigin = { 2.0f, 0.0f };
					s_context.sample->ShiftOrigin( newOrigin );
				}
				else
				{
					s_context.camera.center.x -= 0.5f;
				}
				break;

			case GLFW_KEY_RIGHT:
				// Pan right
				if ( mods == GLFW_MOD_CONTROL )
				{
					b2Vec2 newOrigin = { -2.0f, 0.0f };
					s_context.sample->ShiftOrigin( newOrigin );
				}
				else
				{
					s_context.camera.center.x += 0.5f;
				}
				break;

			case GLFW_KEY_DOWN:
				// Pan down
				if ( mods == GLFW_MOD_CONTROL )
				{
					b2Vec2 newOrigin = { 0.0f, 2.0f };
					s_context.sample->ShiftOrigin( newOrigin );
				}
				else
				{
					s_context.camera.center.y -= 0.5f;
				}
				break;

			case GLFW_KEY_UP:
				// Pan up
				if ( mods == GLFW_MOD_CONTROL )
				{
					b2Vec2 newOrigin = { 0.0f, -2.0f };
					s_context.sample->ShiftOrigin( newOrigin );
				}
				else
				{
					s_context.camera.center.y += 0.5f;
				}
				break;

			case GLFW_KEY_HOME:
				ResetView( &s_context.camera );
				break;

			case GLFW_KEY_R:
				SelectSample( &s_context, s_context.sampleIndex, true );
				break;

			case GLFW_KEY_O:
				s_context.singleStep = true;
				break;

			case GLFW_KEY_P:
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

int main( int, char** )
{
#if defined( _MSC_VER )
	// Enable memory-leak reports
	//_CrtSetBreakAlloc( 217 );
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
#ifdef __APPLE__
		glfwGetMonitorContentScale( primaryMonitor, &s_framebufferScale, &s_framebufferScale );
#else
		float uiScale = 1.0f;
		glfwGetMonitorContentScale( primaryMonitor, &uiScale, &uiScale );
		s_context.uiScale = uiScale;
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

		// s_context.draw.DrawBackground();

		// double cursorPosX = 0, cursorPosY = 0;
		// glfwGetCursorPos( s_context.window, &cursorPosX, &cursorPosY );
		// ImGui_ImplGlfw_CursorPosCallback( s_context.window, cursorPosX / s_windowScale, cursorPosY / s_windowScale );
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		// ImGui_ImplGlfw_CursorPosCallback( s_context.window, cursorPosX / s_windowScale, cursorPosY / s_windowScale );

		ImGuiIO& io = ImGui::GetIO();
		io.DisplaySize.x = s_context.camera.width;
		io.DisplaySize.y = s_context.camera.height;
		io.DisplayFramebufferScale.x = bufferWidth / s_context.camera.width;
		io.DisplayFramebufferScale.y = bufferHeight / s_context.camera.height;

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

		const SampleEntry& entry = g_sampleEntries[s_context.sampleIndex];
		s_context.sample->DrawColoredTextLine( b2_colorYellow, "%s : %s", entry.category, entry.name );

		s_context.sample->Step();

		DrawScreenString( s_context.draw, 5.0f, s_context.camera.height - 10.0f, b2_colorSeaGreen,
						  "%.1f ms - step %d - camera (%g, %g, %g)", 1000.0f * frameTime, s_context.sample->m_stepCount,
						  s_context.camera.center.x, s_context.camera.center.y, s_context.camera.zoom );

		FlushDraw( s_context.draw, &s_context.camera );

		UpdateSampleUI( &s_context );

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
