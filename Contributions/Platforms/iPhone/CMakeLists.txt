cmake_minimum_required(VERSION 2.6)

set(BOX2D_VERSION 2.1.0)
set(BOX2D_BUILD_STATIC true)
set(BOX2D_DIR ../../../Box2D)
subdirs(${BOX2D_DIR}/Box2D)

project(iPhoneTestbed)

include_directories(${BOX2D_DIR} ${BOX2D_DIR}/Testbed/Tests)
file(GLOB iPhoneTestbed_Classes_SRCS Classes/*.mm)
source_group(Classes FILES ${iPhoneTestbed_Classes_SRCS})

set(CMAKE_OSX_SYSROOT iphoneos)
set(CMAKE_OSX_DEPLOYMENT_TARGET "")
set(CMAKE_OSX_ARCHITECTURES $(ARCHS_STANDARD_32_BIT))
set(CMAKE_EXE_LINKER_FLAGS "-framework Foundation -framework CoreGraphics -framework QuartzCore -framework OpenGLES -framework UIKit")
set(MACOSX_BUNDLE_PRODUCT_NAME \${PRODUCT_NAME})
set(MACOSX_BUNDLE_GUI_IDENTIFIER "com.mycompany.\${PRODUCT_NAME:identifier}")

add_executable(iPhoneTestbed MACOSX_BUNDLE
	${iPhoneTestbed_Classes_SRCS}
	main.m
)

target_link_libraries(iPhoneTestbed Box2D)

set_target_properties(iPhoneTestbed PROPERTIES MACOSX_BUNDLE_INFO_PLIST Info.plist.in XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "iPhone Developer")

set(APP_NAME \${TARGET_BUILD_DIR}/\${FULL_PRODUCT_NAME}) 
find_program(IBTOOL ibtool HINTS "/usr/bin" "${OSX_DEVELOPER_ROOT}/usr/bin")
set(NIB MainWindow)
add_custom_command(TARGET iPhoneTestbed POST_BUILD
	COMMAND /Developer/Library/PrivateFrameworks/DevToolsCore.framework/Resources/pbxcp -exclude .DS_Store -exclude CVS -exclude .svn -resolve-src-symlinks Resources/* ${APP_NAME}
	COMMAND ${IBTOOL} --errors --warnings --notices --output-format human-readable-text --compile ${iPhoneTestbed_BINARY_DIR}/\${CONFIGURATION}/${PROJECT_NAME}.app/${NIB}.nib ${NIB}.xib
)
