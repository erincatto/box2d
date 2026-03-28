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

typedef struct Foo
{
	int a;
	float b;
} Foo;

b2DeclareArray( Foo );

typedef struct Bar
{
	b2ArrayC( int ) a;
} Bar;

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

	ENSURE( sum == 100 * 99 / 2 );

	b2Array_Destroy( a );
	return 0;
}

static int TestArrayRemove( void )
{
	b2ArrayC( int16_t ) a = { 0 };

	int n = 10;
	b2Array_Reserve( a, n );
	ENSURE( a.capacity == n && a.count == 0 );

	for ( int i = 0; i < n; ++i )
	{
		b2Array_Push( a, i );
	}

	ENSURE( a.count == n );

	int sum = 0;
	for ( int i = 0; i < n; ++i )
	{
		int temp = b2Array_RemoveSwap( a, 0 );
		sum += temp;
	}

	ENSURE( sum == n * ( n - 1 ) / 2 - 1 );

	b2Array_Destroy( a );
	return 0;
}

static int TestArrayPop( void )
{
	b2ArrayC( uint8_t ) a = { 0 };

	int n = 100;
	b2Array_Resize( a, n );
	ENSURE( a.capacity == n && a.count == n );

	for ( int i = 0; i < n; ++i )
	{
		b2Array_Push( a, i );
	}

	int sum = 0;
	for ( int i = 0; i < n; ++i )
	{
		sum += b2Array_Pop( a );
	}

	ENSURE( sum == 100 * 99 / 2 );

	b2Array_Destroy( a );
	return 0;
}

typedef struct
{
	int index;
} Owner;

b2DeclareArray( Owner );

typedef struct Entity
{
	int bodyIndex;
	int id;
} Entity;

typedef struct Body
{
	int entityIndex;
	float mass;
} Body;

b2DeclareArray( Entity );
b2DeclareArray( Body );

static int TestEmptyArrayProperties( void )
{
	b2ArrayC( int ) a;
	b2Array_Create( a );
	ENSURE( a.count == 0 );
	b2Array_Destroy( a );
	ENSURE( a.count == 0 );
	ENSURE( a.capacity == 0 );
	ENSURE( a.data == NULL );
	return 0;
}

static int TestArrayReserveNoop( void )
{
	b2ArrayC( int ) a = { 0 };
	b2Array_Reserve( a, 16 );
	ENSURE( a.capacity >= 16 );
	int oldCapacity = a.capacity;
	// Reserve with smaller or equal capacity should be a no-op
	b2Array_Reserve( a, 8 );
	ENSURE( a.capacity == oldCapacity );
	b2Array_Reserve( a, oldCapacity );
	ENSURE( a.capacity == oldCapacity );
	b2Array_Destroy( a );
	return 0;
}

static int TestArrayResizeDown( void )
{
	b2ArrayC( int ) a = { 0 };
	for ( int i = 0; i < 10; ++i )
	{
		b2Array_Push( a, i * 10 );
	}
	ENSURE( a.count == 10 );

	b2Array_Resize( a, 5 );
	ENSURE( a.count == 5 );

	// First 5 elements should be unchanged
	for ( int i = 0; i < 5; ++i )
	{
		ENSURE( *b2Array_Get( a, i ) == i * 10 );
	}

	b2Array_Destroy( a );
	return 0;
}

static int TestArrayPopOrder( void )
{
	b2ArrayC( int ) a = { 0 };
	b2Array_Push( a, 10 );
	b2Array_Push( a, 20 );
	b2Array_Push( a, 30 );

	// Pop returns the last element (LIFO)
	ENSURE( b2Array_Pop( a ) == 30 );
	ENSURE( a.count == 2 );
	ENSURE( b2Array_Pop( a ) == 20 );
	ENSURE( a.count == 1 );
	ENSURE( b2Array_Pop( a ) == 10 );
	ENSURE( a.count == 0 );

	b2Array_Destroy( a );
	return 0;
}

static int TestArrayRemoveSwapContents( void )
{
	b2ArrayC( int ) a = { 0 };
	b2Array_Push( a, 100 );
	b2Array_Push( a, 200 );
	b2Array_Push( a, 300 );
	b2Array_Push( a, 400 );

	// Remove middle element: last element swaps into its place
	b2Array_RemoveSwap( a, 1 );
	ENSURE( a.count == 3 );
	ENSURE( *b2Array_Get( a, 0 ) == 100 );
	ENSURE( *b2Array_Get( a, 1 ) == 400 ); // swapped from end
	ENSURE( *b2Array_Get( a, 2 ) == 300 );

	// Remove last element: no swap needed
	b2Array_RemoveSwap( a, 2 );
	ENSURE( a.count == 2 );
	ENSURE( *b2Array_Get( a, 0 ) == 100 );
	ENSURE( *b2Array_Get( a, 1 ) == 400 );

	// Remove first element
	b2Array_RemoveSwap( a, 0 );
	ENSURE( a.count == 1 );
	ENSURE( *b2Array_Get( a, 0 ) == 400 );

	// Remove sole element
	b2Array_RemoveSwap( a, 0 );
	ENSURE( a.count == 0 );

	b2Array_Destroy( a );
	return 0;
}

static int TestArrayGrowthIntegrity( void )
{
	b2ArrayC( int ) a = { 0 };

	// Push many elements to trigger multiple reallocations
	for ( int i = 0; i < 1000; ++i )
	{
		b2Array_Push( a, i );
	}

	ENSURE( a.count == 1000 );
	ENSURE( a.capacity >= 1000 );

	// Verify every element survived the reallocations
	for ( int i = 0; i < 1000; ++i )
	{
		ENSURE( *b2Array_Get( a, i ) == i );
	}

	b2Array_Destroy( a );
	return 0;
}

static int TestArrayInterleavedPushPop( void )
{
	b2ArrayC( int ) a = { 0 };

	b2Array_Push( a, 1 );
	b2Array_Push( a, 2 );
	ENSURE( b2Array_Pop( a ) == 2 );
	ENSURE( a.count == 1 );

	b2Array_Push( a, 3 );
	b2Array_Push( a, 4 );
	ENSURE( a.count == 3 );
	ENSURE( b2Array_Pop( a ) == 4 );
	ENSURE( b2Array_Pop( a ) == 3 );
	ENSURE( b2Array_Pop( a ) == 1 );
	ENSURE( a.count == 0 );

	// Re-use after emptying
	b2Array_Push( a, 99 );
	ENSURE( a.count == 1 );
	ENSURE( *b2Array_Get( a, 0 ) == 99 );

	b2Array_Destroy( a );
	return 0;
}

static int TestArrayEmplaceStruct( void )
{
	b2ArrayC( Foo ) a = { 0 };

	for ( int i = 0; i < 50; ++i )
	{
		Foo* f = b2Array_Emplace( a );
		f->a = i;
		f->b = (float)i * 2.0f;
	}

	ENSURE( a.count == 50 );

	for ( int i = 0; i < 50; ++i )
	{
		Foo* f = b2Array_Get( a, i );
		ENSURE( f->a == i );
		ENSURE( f->b == (float)i * 2.0f );
	}

	b2Array_Destroy( a );
	return 0;
}

static int TestArrayResizeUp( void )
{
	b2ArrayC( int ) a = { 0 };

	b2Array_Resize( a, 10 );
	ENSURE( a.count == 10 );
	ENSURE( a.capacity >= 10 );

	// Write values into resized slots
	for ( int i = 0; i < 10; ++i )
	{
		a.data[i] = i + 1;
	}

	// Resize larger, original values preserved
	b2Array_Resize( a, 20 );
	ENSURE( a.count == 20 );
	for ( int i = 0; i < 10; ++i )
	{
		ENSURE( a.data[i] == i + 1 );
	}

	b2Array_Destroy( a );
	return 0;
}

static int TestArrayPushAfterReserve( void )
{
	b2ArrayC( int ) a = { 0 };

	// Reserve doesn't change count
	b2Array_Reserve( a, 50 );
	ENSURE( a.count == 0 );
	ENSURE( a.capacity >= 50 );

	// Push within reserved capacity (no reallocation expected)
	for ( int i = 0; i < 50; ++i )
	{
		b2Array_Push( a, i * 3 );
	}

	ENSURE( a.count == 50 );
	for ( int i = 0; i < 50; ++i )
	{
		ENSURE( *b2Array_Get( a, i ) == i * 3 );
	}

	b2Array_Destroy( a );
	return 0;
}

static int TestArrayRemoveSwapAllFromEnd( void )
{
	b2ArrayC( int ) a = { 0 };
	b2Array_Push( a, 1 );
	b2Array_Push( a, 2 );
	b2Array_Push( a, 3 );

	// Always remove the last element (no swap path)
	b2Array_RemoveSwap( a, a.count - 1 );
	ENSURE( a.count == 2 );
	ENSURE( *b2Array_Get( a, 0 ) == 1 );
	ENSURE( *b2Array_Get( a, 1 ) == 2 );

	b2Array_RemoveSwap( a, a.count - 1 );
	ENSURE( a.count == 1 );
	ENSURE( *b2Array_Get( a, 0 ) == 1 );

	b2Array_RemoveSwap( a, a.count - 1 );
	ENSURE( a.count == 0 );

	b2Array_Destroy( a );
	return 0;
}

static int TestArraySingleElement( void )
{
	b2ArrayC( Foo ) a = { 0 };

	Foo* f = b2Array_Emplace( a );
	f->a = 7;
	f->b = 3.14f;

	ENSURE( a.count == 1 );
	ENSURE( b2Array_Get( a, 0 )->a == 7 );
	ENSURE( b2Array_Get( a, 0 )->b == 3.14f );

	Foo popped = b2Array_Pop( a );
	ENSURE( popped.a == 7 );
	ENSURE( popped.b == 3.14f );
	ENSURE( a.count == 0 );

	b2Array_Destroy( a );
	return 0;
}

static int TestArrayCreateN( void )
{
	b2ArrayC( int ) a;
	b2Array_CreateN( a, 16 );
	ENSURE( a.count == 0 );
	ENSURE( a.capacity == 16 );
	ENSURE( a.data != NULL );

	// Verify it behaves like a normal array after creation
	for ( int i = 0; i < 16; ++i )
	{
		b2Array_Push( a, i * 5 );
	}

	ENSURE( a.count == 16 );
	for ( int i = 0; i < 16; ++i )
	{
		ENSURE( *b2Array_Get( a, i ) == i * 5 );
	}

	b2Array_Destroy( a );
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
	RUN_SUBTEST( TestEmptyArrayProperties );
	RUN_SUBTEST( TestArrayReserveNoop );
	RUN_SUBTEST( TestArrayResizeDown );
	RUN_SUBTEST( TestArrayResizeUp );
	RUN_SUBTEST( TestArrayPopOrder );
	RUN_SUBTEST( TestArrayRemoveSwapContents );
	RUN_SUBTEST( TestArrayRemoveSwapAllFromEnd );
	RUN_SUBTEST( TestArrayGrowthIntegrity );
	RUN_SUBTEST( TestArrayInterleavedPushPop );
	RUN_SUBTEST( TestArrayEmplaceStruct );
	RUN_SUBTEST( TestArrayPushAfterReserve );
	RUN_SUBTEST( TestArraySingleElement );
	RUN_SUBTEST( TestArrayCreateN );

	return 0;
}
