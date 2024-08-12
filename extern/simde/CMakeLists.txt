# This is a reduced version of https://github.com/simd-everywhere/simde

add_library(
	simde INTERFACE
	check.h
	debug-trap.h
	hedley.h
	simde-aes.h 
	simde-align.h 
	simde-arch.h 
	simde-common.h 
	simde-constify.h 
	simde-detect-clang.h 
	simde-diagnostic.h 
	simde-f16.h 
	simde-features.h 
	simde-math.h
	x86/aes.h 
	x86/avx.h 
	x86/avx2.h 
	x86/f16c.h 
	x86/fma.h 
	x86/mmx.h 
	x86/sse.h 
	x86/sse2.h 
	x86/sse3.h 
	x86/sse4.1.h 
	x86/sse4.2.h 
	x86/ssse3.h 
)

target_include_directories(simde INTERFACE ${CMAKE_CURRENT_SOURCE_DIR})
