// SPDX-FileCopyrightText: 2023 Erin Catto
// SPDX-License-Identifier: MIT

#include "bitset.h"
#include "test_macros.h"

#define COUNT 169

int BitSetTest( void )
{
	b2BitSet bitSet = b2CreateBitSet( COUNT );

	b2SetBitCountAndClear( &bitSet, COUNT );
	bool values[COUNT] = { false };

	int32_t i1 = 0, i2 = 1;
	b2SetBit( &bitSet, i1 );
	values[i1] = true;

	while ( i2 < COUNT )
	{
		b2SetBit( &bitSet, i2 );
		values[i2] = true;
		int32_t next = i1 + i2;
		i1 = i2;
		i2 = next;
	}

	for ( int32_t i = 0; i < COUNT; ++i )
	{
		bool value = b2GetBit( &bitSet, i );
		ENSURE( value == values[i] );
	}

	b2DestroyBitSet( &bitSet );

	return 0;
}
