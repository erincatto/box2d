# glew

add_definitions( -DGLEW_STATIC -D_CRT_SECURE_NO_WARNINGS )

add_library(glew STATIC 
	glew.c
	glew.h
	glxew.h
	wglew.h
)

