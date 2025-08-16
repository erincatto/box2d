# Box2D code generation

set(BOX2D_CODE_GEN_FILES
	main.c
)
add_executable(code_gen ${BOX2D_CODE_GEN_FILES})

set_target_properties(code_gen PROPERTIES
    C_STANDARD 17
    C_STANDARD_REQUIRED YES
    C_EXTENSIONS NO
)

if (BOX2D_COMPILE_WARNING_AS_ERROR)
	set_target_properties(code_gen PROPERTIES COMPILE_WARNING_AS_ERROR ON)
endif()

source_group(TREE ${CMAKE_CURRENT_SOURCE_DIR} PREFIX "" FILES ${BOX2D_CODE_GEN_FILES})

# default startup project for Visual Studio
if(MSVC)
    set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT code_gen)
    set_property(TARGET code_gen PROPERTY VS_DEBUGGER_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
endif()

if(APPLE)
    set_target_properties(code_gen PROPERTIES
        XCODE_GENERATE_SCHEME TRUE
        XCODE_SCHEME_WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}")
endif()
