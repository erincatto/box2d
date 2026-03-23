// SPDX-FileCopyrightText: 2026 Erin Catto
// SPDX-License-Identifier: MIT

#include "container.h"
#include "core.h"
#include "test_macros.h"

#include <assert.h>

b2DeclareArray( int );
b2DeclareArray( uint64_t );
b2DeclareArray( int16_t );
b2DeclareArray( uint8_t );
b2DeclareStackArray( int, 8 );
b2DeclareStackArray( int, 100 );
b2DeclareStackArray( uint64_t, 100 );

typedef struct Foo
{
	int a;
	float b;
} Foo;

b2DeclareArray( Foo );
b2DeclareStackArray( Foo, 1 );

typedef struct Bar
{
	b2ArrayC( int ) a;
} Bar;

typedef struct BarStack
{
	b2StackArray( int, 8 ) a;
} BarStack;

static int TestCreateDestroy( void )
{
	b2ArrayC( int ) a;
	b2Array_Create( a );
	b2Array_Destroy( a );
	return 0;
}

static int TestAccess( void )
{
	b2ArrayC( int ) a;
	b2Array_Create( a );
	b2Array_Push( a, 42 );
	int* element = b2Array_Get( a, 0 );
	ENSURE( *element == 42 );
	b2Array_Destroy( a );
	return 0;
}

static int TestIteration( void )
{
	b2ArrayC( int ) a = { 0 };
	b2Array_Push( a, 1 );
	b2Array_Push( a, 2 );
	b2Array_Push( a, 3 );

	int sum = 0;
	for ( int i = 0; i < a.count; ++i )
	{
		sum += a.data[i];
	}

	ENSURE( sum == 6 );
	b2Array_Destroy( a );
	return 0;
}

static int TestArrayOfStruct( void )
{
	b2ArrayC( Foo ) a;
	b2Array_Create( a );
	b2Array_Push( a, ( (Foo){ .a = 1, .b = 5.0f } ) );
	b2Array_Push( a, ( (Foo){ .a = 2, .b = 6.0f } ) );
	b2Array_Push( a, ( (Foo){ .a = 3, .b = 7.0f } ) );

	int sum1 = 0;
	float sum2 = 0.0f;
	for ( int i = 0; i < a.count; ++i )
	{
		sum1 += a.data[i].a;
		sum2 += a.data[i].b;
	}

	ENSURE( sum1 == 6 );
	ENSURE( sum2 == 18.0f );

	b2Array_Destroy( a );
	return 0;
}

static int TestStructWithArray( void )
{
	Bar a;
	b2Array_Create( a.a );
	b2Array_Push( a.a, 1 );
	b2Array_Push( a.a, 2 );
	b2Array_Push( a.a, 3 );

	int sum1 = 0;
	for ( int i = 0; i < a.a.count; ++i )
	{
		sum1 += a.a.data[i];
	}

	ENSURE( sum1 == 6 );

	b2Array_Destroy( a.a );
	return 0;
}

static int TestArrayEmplace( void )
{
	b2ArrayC( uint64_t ) a = { 0 };

	for ( int i = 0; i < 100; ++i )
	{
		uint64_t* j = b2Array_Emplace( a );
		*j = i;
	}

	int sum = 0;
	for ( int i = 0; i < a.count; ++i )
	{
		sum += a.data[i];
	}

	ENSURE( sum = 100 * 99 );

	b2Array_Destroy( a );
	return 0;
}

static int TestArrayRemove( void )
{
	b2ArrayC( int16_t ) a = { 0 };

	b2Array_Reserve( a, 100 );
	ENSURE( a.capacity == 100 && a.count == 0 );

	for ( int i = 0; i < 100; ++i )
	{
		b2Array_Push( a, i );
	}

	int sum = 0;
	for ( int i = 0; i < a.count; ++i )
	{
		sum += b2Array_RemoveSwap(a, 0);
	}

	ENSURE( sum = 100 * 99 );

	b2Array_Destroy( a );
	return 0;
}

static int TestArrayPop( void )
{
	b2ArrayC( uint8_t ) a = { 0 };

	b2Array_Resize( a, 100 );
	ENSURE( a.capacity == 100 && a.count == 100 );

	for ( int i = 0; i < 100; ++i )
	{
		b2Array_Push( a, i );
	}

	int sum = 0;
	for ( int i = 0; i < a.count; ++i )
	{
		sum += b2Array_Pop( a );
	}

	ENSURE( sum = 100 * 99 );

	b2Array_Destroy( a );
	return 0;
}

static int TestStackAccess( void )
{
	b2StackArray( int, 8 ) a = { 0 };
	b2StackArray_Create( a );
	b2StackArray_Push( a, 42 );
	int* element = b2StackArray_Get( a, 0 );
	ENSURE( *element == 42 );
	b2StackArray_Destroy( a );
	return 0;
}

static int TestStackIteration( void )
{
	b2StackArray( int, 8 ) a = { 0 };
	b2StackArray_Create( a );
	b2StackArray_Push( a, 1 );
	b2StackArray_Push( a, 2 );
	b2StackArray_Push( a, 3 );

	int sum = 0;
	for ( int i = 0; i < a.count; ++i )
	{
		sum += a.data[i];
	}

	ENSURE( sum == 6 );
	b2StackArray_Destroy( a );
	return 0;
}

static int TestStackArrayOfStruct( void )
{
	b2StackArray( Foo, 1 ) a = { 0 };
	b2StackArray_Create( a );
	b2StackArray_Push( a, ( (Foo){ .a = 1, .b = 5.0f } ) );
	b2StackArray_Push( a, ( (Foo){ .a = 2, .b = 6.0f } ) );
	b2StackArray_Push( a, ( (Foo){ .a = 3, .b = 7.0f } ) );

	int sum1 = 0;
	float sum2 = 0.0f;
	for ( int i = 0; i < a.count; ++i )
	{
		sum1 += a.data[i].a;
		sum2 += a.data[i].b;
	}

	ENSURE( sum1 == 6 );
	ENSURE( sum2 == 18.0f );
	b2StackArray_Destroy( a );
	return 0;
}

static int TestStructWithStackArray( void )
{
	BarStack b = { 0 };
	b2StackArray_Create( b.a );
	b2StackArray_Push( b.a, 1 );
	b2StackArray_Push( b.a, 2 );
	b2StackArray_Push( b.a, 3 );

	int sum = 0;
	for ( int i = 0; i < b.a.count; ++i )
	{
		sum += b.a.data[i];
	}

	ENSURE( sum == 6 );
	b2StackArray_Destroy( b.a );
	return 0;
}

static int TestStackRemoveSwap( void )
{
	b2StackArray( int, 8 ) a;
	b2StackArray_Create( a );

	int n = 21;
	for ( int i = 0; i < n; ++i )
	{
		b2StackArray_Push( a, i );
	}

	// Remove all elements by repeatedly swapping index 0 with the tail
	int sum = 0;
	while ( a.count > 0 )
	{
		sum += a.data[0];
		b2StackArray_RemoveSwap( a, 0 );
	}

	ENSURE( sum == n * (n - 1) / 2 );

	b2StackArray_Destroy( a );

	return 0;
}

int ContainerTest( void )
{
	RUN_SUBTEST( TestCreateDestroy );
	RUN_SUBTEST( TestAccess );
	RUN_SUBTEST( TestIteration );
	RUN_SUBTEST( TestArrayOfStruct );
	RUN_SUBTEST( TestStructWithArray );
	RUN_SUBTEST( TestArrayEmplace );
	RUN_SUBTEST( TestArrayRemove );
	RUN_SUBTEST( TestArrayPop );

	RUN_SUBTEST( TestStackAccess );
	RUN_SUBTEST( TestStackIteration );
	RUN_SUBTEST( TestStackArrayOfStruct );
	RUN_SUBTEST( TestStructWithStackArray );
	RUN_SUBTEST( TestStackRemoveSwap );

	return 0;
}
