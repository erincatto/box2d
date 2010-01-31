add_definitions( -DFREEGLUT_STATIC -D_CRT_SECURE_NO_WARNINGS )

set(glui
	algebra3.cpp
	arcball.cpp
	glui_add_controls.cpp
	glui_bitmap_img_data.cpp
	glui_bitmaps.cpp
	glui_button.cpp
	glui_checkbox.cpp
	glui_column.cpp
	glui_commandline.cpp
	glui_control.cpp
	glui_edittext.cpp
	glui_filebrowser.cpp
	glui_list.cpp
	glui_listbox.cpp
	glui_mouse_iaction.cpp
	glui_node.cpp
	glui_panel.cpp
	glui_radio.cpp
	glui_rollout.cpp
	glui_rotation.cpp
	glui_scrollbar.cpp
	glui_separator.cpp
	glui_spinner.cpp
	glui_statictext.cpp
	glui_string.cpp
	glui_textbox.cpp
	glui_translation.cpp
	glui_tree.cpp
	glui_treepanel.cpp
	glui_window.cpp
	glui.cpp
	quaternion.cpp
)

include_directories (
	${OPENGL_INCLUDE_DIR}
	../freeglut
)

add_library(glui
	${glui}
)

