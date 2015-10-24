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

class CommonTests : public ::testing::Test {
		protected:
	virtual void SetUp();
	virtual void TearDown();
};

void
CommonTests::SetUp()
{
	// Intentionally blank.
}
void
CommonTests::TearDown()
{
	// Intentionally blank.
}

static time_t mock_sec;
static long   mock_nsec;

static const int NSEC_PER_SEC = 1000000000;

// SystemGetTimeFunc type definitions must match b2Timer.cpp
#if defined(_WIN32)

#include <windows.h>

typedef BOOL (WINAPI *SystemGetTimeFunc)(_Out_ LARGE_INTEGER *lpFrequency);
extern SystemGetTimeFunc systemGetTimeFunc;
extern SystemGetTimeFunc systemGetFreqFunc;

static BOOL WINAPI MockSystemTime(_Out_ LARGE_INTEGER *lpFrequency)
{
	lpFrequency->QuadPart = ((int64)mock_sec) * NSEC_PER_SEC + mock_nsec;
	return TRUE;
}

static BOOL WINAPI MockSystemFreq(_Out_ LARGE_INTEGER *lpFrequency)
{
	// mock frequency 1,000,000,000 ticks per second
	lpFrequency->QuadPart = NSEC_PER_SEC;
	return TRUE;
}

#elif defined(__linux__)

typedef int (*SystemGetTimeFunc)(clockid_t clk_id, struct timespec *tp);
extern SystemGetTimeFunc systemGetTimeFunc;

static int MockSystemTime(clockid_t clk_id, struct timespec *tp)
{
	tp->tv_sec = mock_sec;
	tp->tv_nsec = mock_nsec;
	return 0;
}

#elif defined(__APPLE__)

typedef int (*SystemGetTimeFunc)(struct timeval * tp, void * tzp);
extern SystemGetTimeFunc systemGetTimeFunc;

int MockSystemTime(struct timeval * tp, void * tzp)
{
	tp->tv_sec = mock_sec;
	tp->tv_usec = mock_nsec/1000;
	return 0;
}

#endif

TEST_F(CommonTests, b2Timer) {

#ifdef _WIN32
	systemGetFreqFunc = MockSystemFreq;
#endif
	systemGetTimeFunc = MockSystemTime;
	mock_sec = 123456789;
	mock_nsec = 987654321;

	b2Timer timer;
	EXPECT_FLOAT_EQ(0.0f,timer.GetMilliseconds());

	// 1us is the lowest resolution that's supported on all platforms
	mock_nsec += 1000; // add 1us
	EXPECT_FLOAT_EQ(0.001f,timer.GetMilliseconds());

	mock_sec += 1;
	EXPECT_FLOAT_EQ(1000.001f,timer.GetMilliseconds());

	mock_nsec += 10000; // add 10us
	EXPECT_FLOAT_EQ(1000.011f,timer.GetMilliseconds());

	// add 1/10 of a second, 100ms
	// this should cause nsec to exceed NSEC_PER_SEC
	mock_nsec += NSEC_PER_SEC/10;

	// wrap nsec
	while (mock_nsec >= NSEC_PER_SEC)
	{
		mock_nsec -= NSEC_PER_SEC;
		mock_sec++;
	}
	EXPECT_FLOAT_EQ(1100.011f,timer.GetMilliseconds());

	timer.Reset();
	EXPECT_FLOAT_EQ(0.0f,timer.GetMilliseconds());
}

TEST_F(CommonTests, b2Stat) {
	b2Stat stat;
	EXPECT_EQ(0,stat.GetCount());
	EXPECT_EQ(FLT_MAX,stat.GetMin());
	EXPECT_EQ(-FLT_MAX,stat.GetMax());
	EXPECT_EQ(0.0f,stat.GetMean());

	stat.Record(10.0f);
	EXPECT_EQ(1,stat.GetCount());
	EXPECT_FLOAT_EQ(10.0f,stat.GetMin());
	EXPECT_FLOAT_EQ(10.0f,stat.GetMax());
	EXPECT_FLOAT_EQ(10.0f,stat.GetMean());

	stat.Record(1.0f);
	stat.Record(100.0f);
	EXPECT_EQ(3,stat.GetCount());
	EXPECT_FLOAT_EQ(1.0f,stat.GetMin());
	EXPECT_FLOAT_EQ(100.0f,stat.GetMax());
	EXPECT_FLOAT_EQ(111.0f/3.0f,stat.GetMean());

	stat.Clear();
	EXPECT_EQ(0,stat.GetCount());
	EXPECT_EQ(FLT_MAX,stat.GetMin());
	EXPECT_EQ(-FLT_MAX,stat.GetMax());
	EXPECT_EQ(0.0f,stat.GetMean());
}

TEST_F(CommonTests, b2IsValid) {
	const float32 one = 1.0f;
	const float32 inf = std::numeric_limits<float32>::infinity();
	const float32 neginf = -1.0f * inf;
	const float32 nan = inf * 0.0f;
	EXPECT_TRUE(b2IsValid(one));
	EXPECT_FALSE(b2IsValid(inf));
	EXPECT_FALSE(b2IsValid(neginf));
	EXPECT_FALSE(b2IsValid(nan));
}

int
main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
