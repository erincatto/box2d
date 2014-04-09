-- Box2D premake script.
-- http://industriousone.com/premake

local action = _ACTION or ""

solution "Box2D"
	location ( "Build/" .. action )
	configurations { "Debug", "Release" }
	
	configuration "vs*"
		defines { "_CRT_SECURE_NO_WARNINGS" }	
		
	configuration "Debug"
		targetdir ( "Build/" .. action .. "/bin/Debug" )
		flags { "Symbols" }

   configuration "Release"
		targetdir ( "Build/" .. action .. "/bin/Release" )
		defines { "NDEBUG" }
		flags { "Optimize" }

	project "Box2D"
		kind "StaticLib"
		language "C++"
		files { "Box2D/**.h", "Box2D/**.cpp" }
		vpaths { [""] = "Box2D" }
		includedirs { "." }
		
	project "GLEW"
		kind "StaticLib"
		language "C++"
		files { "glew/*.h", "glew/*.c" }
		vpaths { ["Headers"] = "**.h",  ["Sources"] = "**.c" }
		includedirs { "." }
			 
	project "GLFW"
		kind "StaticLib"
		language "C"
		files { "glfw/*.h", "glfw/*.c" }
		vpaths { ["Headers"] = "**.h",  ["Sources"] = "**.c" }
	
	project "HelloWorld"
		kind "ConsoleApp"
		language "C++"
		files { "HelloWorld/HelloWorld.cpp" }
		vpaths { [""] = "HelloWorld" }
		includedirs { "." }
		links { "Box2D" }

	project "Testbed"
		kind "ConsoleApp"
		language "C++"
		files { "Testbed/**.h", "Testbed/**.cpp" }
		vpaths { [""] = "Testbed" }
		includedirs { "." }
		links { "Box2D", "GLEW", "GLFW" }
		configuration { "windows" }
			links { "glu32", "opengl32", "winmm" }
		configuration { "macosx" }
			linkoptions { "-framework OpenGL -framework Cocoa" }
		configuration { "not windows", "not macosx" }
			links { "X11", "GL", "GLU" }

