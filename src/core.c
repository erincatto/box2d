// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "core.h"

#include "box2d/math_functions.h"

#include <stdio.h>

float b2_lengthUnitsPerMeter = 1.0f;

void b2SetLengthUnitsPerMeter( float lengthUnits )
{
	B2_ASSERT( b2IsValid( lengthUnits ) && lengthUnits > 0.0f );
	b2_lengthUnitsPerMeter = lengthUnits;
}

float b2GetLengthUnitsPerMeter( void )
{
	return b2_lengthUnitsPerMeter;
}

static int b2DefaultAssertFcn( const char* condition, const char* fileName, int lineNumber )
{
	printf( "BOX2D ASSERTION: %s, %s, line %d\n", condition, fileName, lineNumber );

	// return non-zero to break to debugger
	return 1;
}

b2AssertFcn* b2AssertHandler = b2DefaultAssertFcn;

void b2SetAssertFcn( b2AssertFcn* assertFcn )
{
	B2_ASSERT( assertFcn != NULL );
	b2AssertHandler = assertFcn;
}

b2Version b2GetVersion( void )
{
	return ( b2Version ){ 3, 0, 0 };
}

#if 0
#if defined( B2_CPU_ARM )

b2InstructionSet b2GetInstructionSet()
{
	return B2_NEON;
}

#elif defined( B2_CPU_WASM )

b2InstructionSet b2GetInstructionSet()
{
	return B2_SSE2;
}

#else

	#ifdef _WIN32
		#include <intrin.h>
	#endif

static inline void b2CPUId( uint32_t* eax, uint32_t* ebx, uint32_t* ecx, uint32_t* edx )
{
	#if defined( _MSC_VER )
	int cpu_info[4];
	__cpuidex( cpu_info, *eax, *ecx );
	*eax = cpu_info[0];
	*ebx = cpu_info[1];
	*ecx = cpu_info[2];
	*edx = cpu_info[3];
	#else
	uint32_t a = *eax, b, c = *ecx, d;
	asm volatile( "cpuid\n\t" : "+a"( a ), "=b"( b ), "+c"( c ), "=d"( d ) );
	*eax = a;
	*ebx = b;
	*ecx = c;
	*edx = d;
	#endif
}

static const uint32_t cpuid_avx2_bit = 1 << 5;	 // bit 5 of EBX for EAX=0x7
static const uint32_t cpuid_sse42_bit = 1 << 20; //  bit 20 of ECX for EAX=0x1

b2InstructionSet b2GetInstructionSet()
{
	uint32_t eax, ebx, ecx, edx;

	// EBX for EAX=0x1
	eax = 0x1;
	ecx = 0x0;
	b2CPUId( &eax, &ebx, &ecx, &edx );

	if ( ( ecx & cpuid_sse42_bit ) == 0 )
	{
		// No SSE4.2 so use SSE2
		return B2_SSE2;
	}

	// ECX for EAX=0x7
	eax = 0x7;
	ecx = 0x0;
	b2CPUId( &eax, &ebx, &ecx, &edx );
	if ( ebx & cpuid_avx2_bit )
	{
		return B2_AVX2;
	}

	// Even if the CPU supports AVX, just use SSE2
	return B2_SSE2;
}

#endif
#endif
