/*
* Copyright (c) 2013 Google, Inc.
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/
#include "gtest/gtest.h"
#include "Box2D/Box2D.h"
#include <limits>
#include <iostream>
#include <vector>

// Overrides the default allocators to track allocations and frees.
class MockAllocator {
public:
	// Initialize the allocation and free counters.
	MockAllocator() : m_allocCount(0), m_freeCount(0)
	{
	}

	// Disable the override of the allocation functions.
	~MockAllocator()
	{
		Disable();
	}

	// Get the number of times Alloc has been called.
	uint32 GetAllocCount() const { return m_allocCount; }
	// Get the number of times Free has been called.
	uint32 GetFreeCount() const { return m_freeCount; }

	// Override b2Alloc and b2Free.
	void Enable()
	{
		b2SetAllocFreeCallbacks(Alloc, Free, this);
	}

	// Restore b2Alloc and b2Free to their default behavior.
	void Disable()
	{
		b2SetAllocFreeCallbacks(NULL, NULL, NULL);
	}

	// Reset the allocation and free counters and restore the default
	// allocators.
	void Reset()
	{
		Disable();
		m_allocCount = 0;
		m_freeCount = 0;
	}

protected:
	uint32 m_allocCount;
	uint32 m_freeCount;

public:
	// Increment the allocation counter and allocate size bytes of memory.
	static void* Alloc(int32 size, void* callbackData)
	{
		MockAllocator* const allocator = (MockAllocator*)callbackData;
		allocator->m_allocCount++;
		return malloc(size);
	}

	// Decrement the allocation counter and free the specified memory.
	static void Free(void* mem, void* callbackData)
	{
		MockAllocator* const allocator = (MockAllocator*)callbackData;
		allocator->m_freeCount++;
		free(mem);
	}
};

class BlockAllocatorTests : public ::testing::Test {
protected:
	virtual void SetUp();
	virtual void TearDown();

protected:
	MockAllocator m_mockAllocator;
};

void
BlockAllocatorTests::SetUp()
{
	// Intentionally blank.
}
void
BlockAllocatorTests::TearDown()
{
	// Reset the mock allocator.
	m_mockAllocator.Reset();
}


// Memory block sizes to allocate
static const int32 ALLOCATION_SIZES[] = { 1, 10, 100, 1000, 10000 };

// Number of times to iterate through sizes above
static const int32 ALLOCATION_ITERATIONS[] = { 1, 2, 3, 10, 100 };

// Number of times in-a-row to repeat each allocation size
static const int32 ALLOCATION_RUN_LENGTHS[] = { 1, 2, 13, 22 };

// Prime numbers, used to re-order the allocation sizes
static const int32 ALLOCATION_PRIMES[] = { 1, 2, 3 };

// How to interleave allocations and frees; number of allocs/frees in a row
static const int32 ALLOCATION_FREE_RUNS[] = { 1, 2, 100000 };

static const int32 NUM_ALLOCATION_SIZES =
	sizeof(ALLOCATION_SIZES) / sizeof(ALLOCATION_SIZES[0]);
static const int32 NUM_ALLOCATION_ITERATIONS =
	sizeof(ALLOCATION_ITERATIONS) / sizeof(ALLOCATION_ITERATIONS[0]);
static const int32 NUM_ALLOCATION_RUN_LENGTHS =
	sizeof(ALLOCATION_RUN_LENGTHS) / sizeof(ALLOCATION_RUN_LENGTHS[0]);
static const int32 NUM_ALLOCATION_PRIMES =
	sizeof(ALLOCATION_PRIMES) / sizeof(ALLOCATION_PRIMES[0]);
static const int32 NUM_ALLOCATION_FREE_RUNS =
	sizeof(ALLOCATION_FREE_RUNS) / sizeof(ALLOCATION_FREE_RUNS[0]);
static const int32 MAX_ALLOCATION_ITERATIONS =
	ALLOCATION_ITERATIONS[NUM_ALLOCATION_ITERATIONS - 1];
static const int32 MAX_ALLOCATION_RUN_LENGTH =
	ALLOCATION_RUN_LENGTHS[NUM_ALLOCATION_RUN_LENGTHS - 1];
static const int32 MAX_ALLOCATIONS = MAX_ALLOCATION_ITERATIONS
									 * NUM_ALLOCATION_SIZES
									 * MAX_ALLOCATION_RUN_LENGTH;

static inline int32 Min(int32 a, int32 b)
{
	return a < b ? a : b;
}

// Pre-calculate an array of allocation sizes
// We allocate and free items in this order
static int32 CalcAllocationSizes(
	int32 iterations,
	int32 runLength,
	int32 prime,
	std::vector<int32>* const sizes,
	int32 maxSizes)
{
	int32 numSizes = 0;
	for (int i = 0; i < iterations; ++i) {
		for (int j = 0; j < NUM_ALLOCATION_SIZES; ++j) {
			const int32 allocIdx = prime * j % NUM_ALLOCATION_SIZES;
			const int32 size = ALLOCATION_SIZES[allocIdx];

			for (int k = 0; k < runLength; ++k) {
				if (numSizes >= maxSizes)
					return numSizes;

				(*sizes)[numSizes++] = size;
			}
		}
	}

	return numSizes;
}

// Allocate 'freeRun' memory blocks with size from 'sizes', then free them.
// Repeat until 'numSizes' memory blocks have been allocated and freed.
static void AllocateAndFree(
	b2BlockAllocator* alloc,
	const std::vector<int32>& sizes,
	int32 numSizes,
	int32 freeRun)
{
	std::vector<void*> mem(MAX_ALLOCATIONS);

	// Make a bunch of allocations
	int numAlloc = 0;
	while (numAlloc < numSizes) {
		const int32 run = Min(freeRun, numSizes - numAlloc);

		// Allocate 'run' memory blocks. Get size from 'sizes'.
		for (int i = 0; i < run; ++i) {
			mem[i] = alloc->Allocate(sizes[numAlloc + i]);
		}

		// Free the memory blocks allocated above.
		for (int i = 0; i < run; ++i) {
			alloc->Free(mem[i], sizes[numAlloc + i]);
		}

		numAlloc += run;
	}
}


TEST_F(BlockAllocatorTests, b2BlockAllocator_FreesAllMallocs)
{
	const int32 startAllocs = b2GetNumAllocs();
	std::vector<int32> sizes(MAX_ALLOCATIONS);

	for (int i = 0; i < NUM_ALLOCATION_ITERATIONS; ++i) {
		for (int j = 0; j < NUM_ALLOCATION_RUN_LENGTHS; ++j) {
			for (int k = 0; k < NUM_ALLOCATION_PRIMES; ++k) {

				const int32 numSizes = CalcAllocationSizes(
					ALLOCATION_ITERATIONS[i],
					ALLOCATION_RUN_LENGTHS[j],
					ALLOCATION_PRIMES[k],
					&sizes,
					MAX_ALLOCATIONS);

				for (int m = 0; m < NUM_ALLOCATION_FREE_RUNS; ++m) {
					b2BlockAllocator* alloc = new b2BlockAllocator;

					AllocateAndFree(
						alloc,
						sizes,
						numSizes,
						ALLOCATION_FREE_RUNS[m]);

					delete alloc;

					// After allocating and freeing blocks, the number of
					// global mallocs should be the same as when we started
					EXPECT_EQ(startAllocs, b2GetNumAllocs());
				}
			}
		}
	}
}

// Verify that it's possible to override the default memory allocators.
TEST_F(BlockAllocatorTests, OverrideAllocFree)
{
	m_mockAllocator.Enable();
	void *a = b2Alloc(1);
	void *b = b2Alloc(2);
	void *c = b2Alloc(4);
	EXPECT_EQ(3U, m_mockAllocator.GetAllocCount());
	EXPECT_EQ(0U, m_mockAllocator.GetFreeCount());
	b2Free(a);
	b2Free(b);
	b2Free(c);
	EXPECT_EQ(3U, m_mockAllocator.GetAllocCount());
	EXPECT_EQ(3U, m_mockAllocator.GetFreeCount());
}


// Verify that disabling the default memory allocator restores malloc/free.
TEST_F(BlockAllocatorTests, RestoreMallocFree)
{
	m_mockAllocator.Enable();
	void *mem = b2Alloc(1);
	// Reset the global allocated block counter so the default allocator can
	// be restored without causing an assert.
	b2SetNumAllocs(0);
	m_mockAllocator.Disable();
	// Set the allocation counter so the free will decrement it to 0.
	b2SetNumAllocs(1);
	b2Free(mem);
	EXPECT_EQ(1U, m_mockAllocator.GetAllocCount());
	EXPECT_EQ(0U, m_mockAllocator.GetFreeCount());
}

int
main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
