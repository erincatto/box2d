# glfw

add_definitions( -DGLFW_STATIC -D_CRT_SECURE_NO_WARNINGS )

if(APPLE)
set(PLATFORM_SRC
	egl_context.c
	egl_platform.h
	nsgl_context.m
	nsgl_platform.h
	cocoa_clipboard.m
	cocoa_gamma.c
	cocoa_init.m
	cocoa_joystick.m
	cocoa_monitor.m
	cocoa_platform.h
	cocoa_time.c
	cocoa_window.m
)
elseif(WIN32)
set(PLATFORM_SRC
	wglext.h
	wgl_context.c
	wgl_platform.h
	win32_clipboard.c
	win32_gamma.c
	win32_init.c
	win32_joystick.c
	win32_monitor.c
	win32_platform.h
	win32_time.c
	win32_window.c
)
elseif(LINUX)
set(PLATFORM_SRC
	glext.h
	glfw3.h
	glfw3native.h
	glxext.h
	glx_context.c
	glx_platform.h
	x11_clipboard.c
	x11_gamma.c
	x11_init.c
	x11_joystick.c
	x11_monitor.c
	x11_platform.h
	x11_time.c
	x11_unicode.c
	x11_window.c
)
endif(APPLE)

add_library(glfw STATIC
	clipboard.c
	config.h
	context.c
	gamma.c
	init.c
	input.c
	internal.h
	joystick.c
	monitor.c
	time.c
	window.c
	${PLATFORM_SRC}
)

