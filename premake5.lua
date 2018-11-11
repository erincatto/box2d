-- Box2D premake5 script.
-- https://premake.github.io/

workspace 'Box2D'
	configurations { 'Debug', 'Release' }
	startproject 'Testbed'
	location 'Build'
	symbols 'On'
	warnings 'Extra'
    cppdialect 'C++11'

    filter 'system:linux'
        platforms { 'x86_64' }
    filter 'system:macosx'
        platforms { 'x86_64' }
    filter 'system:windows'
        platforms { 'x86', 'x86_64' }
        defaultplatform 'x86_64'
		defines { '_CRT_SECURE_NO_WARNINGS' }
	filter {}

	filter 'configurations:Debug'
	 	defines { 'DEBUG' }
		optimize 'Off'
	filter 'configurations:Release'
		defines { 'NDEBUG' }
		optimize 'On'
	filter {}

project 'Box2D'
	kind 'StaticLib'
	files { 'Box2D/**' }
	includedirs { '.' }

project 'HelloWorld'
	kind 'ConsoleApp'
	files { 'HelloWorld/HelloWorld.cpp' }
	includedirs { '.' }
	links { 'Box2D' }

project 'Testbed'
	kind 'ConsoleApp'
	debugdir 'Testbed'
	warnings 'Default'
	includedirs { '.' }

	files
	{
		'Testbed/Data/*',
		'Testbed/Framework/*',
		'Testbed/Tests/*',
		'Testbed/glfw/internal.h',
		'Testbed/glfw/glfw_config.h',
		'Testbed/glfw/glfw3.h',
		'Testbed/glfw/glfw3native.h',
		'Testbed/glfw/context.c',
		'Testbed/glfw/init.c',
		'Testbed/glfw/input.c',
		'Testbed/glfw/monitor.c',
		'Testbed/glfw/vulkan.c',
		'Testbed/glfw/window.c',
		'Testbed/imgui/*'
	}

    filter { 'system:windows' }
    	files
    	{ 
    		'Testbed/glad/*',
			'Testbed/glfw/win32_platform.h',
			'Testbed/glfw/win32_joystick.h',
			'Testbed/glfw/wgl_context.h',
			'Testbed/glfw/egl_context.h',
			'Testbed/glfw/win32_init.c',
			'Testbed/glfw/win32_joystick.c',
			'Testbed/glfw/win32_monitor.c',
			'Testbed/glfw/win32_time.c',
			'Testbed/glfw/win32_tls.c',
			'Testbed/glfw/win32_window.c',
			'Testbed/glfw/wgl_context.c',
			'Testbed/glfw/egl_context.c'
		}
    	links { 'Box2D', 'opengl32', 'winmm' }

    filter { 'system:macosx' }
    	files
		{
			'Testbed/glfw/cocoa_platform.h',
			'Testbed/glfw/iokit_joystick.h',
			'Testbed/glfw/posix_tls.h',
			'Testbed/glfw/nsgl_context.h',
			'Testbed/glfw/egl_context.h',
			'Testbed/glfw/cocoa_init.m',
			'Testbed/glfw/cocoa_joystick.m',
			'Testbed/glfw/cocoa_monitor.m',
			'Testbed/glfw/cocoa_window.m',
			'Testbed/glfw/cocoa_time.c',
			'Testbed/glfw/posix_tls.c',
			'Testbed/glfw/nsgl_context.m',
			'Testbed/glfw/egl_context.c'
		}
		--defines { 'GLFW_INCLUDE_GLCOREARB' }
		links
		{
			'Box2D',
			'OpenGL.framework',
			'Cocoa.framework',
			'IOKit.framework',
			'CoreFoundation.framework',
			'CoreVideo.framework'
		}
    
    filter { 'system:linux' }
    	files
    	{
    		'Testbed/glad/*',
    		'Testbed/glfw/x11_platform.h',
			'Testbed/glfw/xkb_unicode.h',
			'Testbed/glfw/linux_joystick.h',
			'Testbed/glfw/posix_time.h',
			'Testbed/glfw/posix_tls.h',
			'Testbed/glfw/glx_context.h',
			'Testbed/glfw/egl_context.h',
			'Testbed/glfw/x11_init.c',
			'Testbed/glfw/x11_monitor.c',
			'Testbed/glfw/x11_window.c',
			'Testbed/glfw/glx_context.h',
			'Testbed/glfw/glx_context.c',
			'Testbed/glfw/glext.h',
			'Testbed/glfw/xkb_unicode.c',
			'Testbed/glfw/linux_joystick.c',
			'Testbed/glfw/posix_time.c',
			'Testbed/glfw/posix_tls.c',
			'Testbed/glfw/glx_context.c',
			'Testbed/glfw/egl_context.c'
		}
		links
		{
			'Box2D',
			'GL',
			'X11',
			'Xrandr',
			'Xinerama',
			'Xcursor',
			'pthread',
			'dl'
		}
	
	filter {}
