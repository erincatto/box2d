// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "test_macros.h"

#include "box2d/id.h"

int IdTest( void )
{
	uint32_t a = 0x01234567;

	{
		b2WorldId id = b2LoadWorldId( a );
		uint32_t b = b2StoreWorldId( id );
		ENSURE( b == a );
	}

	uint64_t x = 0x0123456789ABCDEFull;

	{
		b2BodyId id = b2LoadBodyId( x );
		uint64_t y = b2StoreBodyId( id );
		ENSURE( x == y );
	}

	{
		b2ShapeId id = b2LoadShapeId( x );
		uint64_t y = b2StoreShapeId( id );
		ENSURE( x == y );
	}

	{
		b2ChainId id = b2LoadChainId( x );
		uint64_t y = b2StoreChainId( id );
		ENSURE( x == y );
	}

	{
		b2JointId id = b2LoadJointId( x );
		uint64_t y = b2StoreJointId( id );
		ENSURE( x == y );
	}

	return 0;
}
