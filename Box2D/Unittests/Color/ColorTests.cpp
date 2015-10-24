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
#include "Box2D/Box2D.h"
#include "gtest/gtest.h"

#include "TestCommon.h"

// Epsilon values to account for rounding error in conversions between integer
// and floating point values.
const float32 FLOAT_COLOR_EPSILON = 0.01f;
const uint8 INT8_COLOR_EPSILON = 1;

class ColorTests : public ::testing::Test {
};

// Construct a particle color with black and nothing in the alpha channel.
TEST_F(ColorTests, ConstructZero) {
	b2ParticleColor color(0, 0, 0, 0);
	EXPECT_EQ(0, color.r);
	EXPECT_EQ(0, color.g);
	EXPECT_EQ(0, color.b);
	EXPECT_EQ(0, color.a);
	EXPECT_TRUE(color == b2ParticleColor_zero);
}

// Construct an opaque salmon particle color.
TEST_F(ColorTests, ConstructSalmon) {
	b2ParticleColor color(0xfa, 0x80, 0x72, 0xff);
	EXPECT_EQ(0xfa, color.r);
	EXPECT_EQ(0x80, color.g);
	EXPECT_EQ(0x72, color.b);
	EXPECT_EQ(0xff, color.a);
}

// Construct an opaque salmon particle color from a Box2D color.
TEST_F(ColorTests, ConstructFromb2Color) {
	b2Color color(0.98f, 0.5f, 0.45f);
	b2ParticleColor particleColor(color);
	EXPECT_NEAR(0xfa, particleColor.r, INT8_COLOR_EPSILON);
	EXPECT_NEAR(0x80, particleColor.g, INT8_COLOR_EPSILON);
	EXPECT_NEAR(0x72, particleColor.b, INT8_COLOR_EPSILON);
	EXPECT_NEAR(0xff, particleColor.a, INT8_COLOR_EPSILON);
}

// Verify IsZero() is true for a zero color.
TEST_F(ColorTests, IsZero) {
	b2ParticleColor zeroColor(0, 0, 0, 0);
	EXPECT_TRUE(zeroColor.IsZero());
	b2ParticleColor darkRedColor(1, 0, 0, 0);
	EXPECT_FALSE(darkRedColor.IsZero());
	b2ParticleColor darkGreenColor(0, 1, 0, 0);
	EXPECT_FALSE(darkGreenColor.IsZero());
	b2ParticleColor darkBlueColor(0, 0, 1, 0);
	EXPECT_FALSE(darkBlueColor.IsZero());
	b2ParticleColor blackWithAlpha(0, 0, 0, 1);
	EXPECT_FALSE(blackWithAlpha.IsZero());
}

// Set a color using integer arguments.
TEST_F(ColorTests, SetViolet) {
	b2ParticleColor color;
	color.Set(0x8d, 0x38, 0xc9, 0xff);
	EXPECT_EQ(0x8d, color.r);
	EXPECT_EQ(0x38, color.g);
	EXPECT_EQ(0xc9, color.b);
	EXPECT_EQ(0xff, color.a);
}

// Set a color using a Box2D color.
TEST_F(ColorTests, Setb2Color) {
	b2Color color(0.55f, 0.22f, 0.79f);
	b2ParticleColor particleColor;
	particleColor.Set(color);
	EXPECT_NEAR(0x8d, particleColor.r, INT8_COLOR_EPSILON);
	EXPECT_NEAR(0x38, particleColor.g, INT8_COLOR_EPSILON);
	EXPECT_NEAR(0xc9, particleColor.b, INT8_COLOR_EPSILON);
	EXPECT_NEAR(0xff, particleColor.a, INT8_COLOR_EPSILON);
}

// Get a Box2D color from a particle color.
TEST_F(ColorTests, GetColor) {
	b2ParticleColor particleColor(0x8d, 0x38, 0xc9, 0xff);
	b2Color color = particleColor.GetColor();
	EXPECT_NEAR(0.55f, color.r, FLOAT_COLOR_EPSILON);
	EXPECT_NEAR(0.22f, color.g, FLOAT_COLOR_EPSILON);
	EXPECT_NEAR(0.79f, color.b, FLOAT_COLOR_EPSILON);
}

// Copy a particle color.
TEST_F(ColorTests, Copy) {
	b2ParticleColor colorA(0xfa, 0x80, 0x72, 0xff);
	b2ParticleColor colorB;
	colorB = colorA;
	EXPECT_EQ(0xfa, colorB.r);
	EXPECT_EQ(0x80, colorB.g);
	EXPECT_EQ(0x72, colorB.b);
	EXPECT_EQ(0xff, colorB.a);
}

// In place multiplication of a color by a scalar floating point value.
TEST_F(ColorTests, ScaleInPlaceByFloat) {
	b2ParticleColor color(0x11, 0x22, 0x33, 0x44);
	color *= 2.0f;
	EXPECT_EQ(0x22, color.r);
	EXPECT_EQ(0x44, color.g);
	EXPECT_EQ(0x66, color.b);
	EXPECT_EQ(0x88, color.a);
}

// In place scaling of a color by a scalar integer value.
TEST_F(ColorTests, ScaleInPlaceByInteger) {
	b2ParticleColor color(0x22, 0x44, 0x66, 0x88);
	// 128 is equivalent to 0.5f.
	color *= (uint8)128;
	EXPECT_EQ(0x11, color.r);
	EXPECT_EQ(0x22, color.g);
	EXPECT_EQ(0x33, color.b);
	EXPECT_EQ(0x44, color.a);
}

// Scale color by a floating point value.
TEST_F(ColorTests, ScaleByFloat) {
	const b2ParticleColor colorSource(0x22, 0x44, 0x66, 0x88);
	b2ParticleColor colorTarget = colorSource * 0.5f;
	EXPECT_EQ(0x11, colorTarget.r);
	EXPECT_EQ(0x22, colorTarget.g);
	EXPECT_EQ(0x33, colorTarget.b);
	EXPECT_EQ(0x44, colorTarget.a);
}

// Scale color by an integer.
TEST_F(ColorTests, ScaleByInteger) {
	const b2ParticleColor colorSource(0x22, 0x44, 0x66, 0x88);
	b2ParticleColor colorTarget = colorSource * (uint8)128;
	EXPECT_EQ(0x11, colorTarget.r);
	EXPECT_EQ(0x22, colorTarget.g);
	EXPECT_EQ(0x33, colorTarget.b);
	EXPECT_EQ(0x44, colorTarget.a);
}

// In place addition of one color with another.
TEST_F(ColorTests, InPlaceAdditionOfColors) {
	const b2ParticleColor rhs(0x44, 0x33, 0x22, 0x11);
	b2ParticleColor lhs(0x33, 0x22, 0x11, 0x77);
	lhs += rhs;
	EXPECT_EQ(0x77, lhs.r);
	EXPECT_EQ(0x55, lhs.g);
	EXPECT_EQ(0x33, lhs.b);
	EXPECT_EQ(0x88, lhs.a);
}

// Addition of two colors.
TEST_F(ColorTests, AdditionOfColors) {
	const b2ParticleColor colorA(0x44, 0x33, 0x22, 0x11);
	const b2ParticleColor colorB(0x33, 0x22, 0x11, 0x77);
	b2ParticleColor result = colorA + colorB;
	EXPECT_EQ(0x77, result.r);
	EXPECT_EQ(0x55, result.g);
	EXPECT_EQ(0x33, result.b);
	EXPECT_EQ(0x88, result.a);
}

// In place subtraction of one color from another.
TEST_F(ColorTests, InPlaceSubtractionOfColors) {
	const b2ParticleColor rhs(0x12, 0x13, 0x21, 0x01);
	b2ParticleColor lhs(0x44, 0x33, 0x22, 0x11);
	lhs -= rhs;
	EXPECT_EQ(0x32, lhs.r);
	EXPECT_EQ(0x20, lhs.g);
	EXPECT_EQ(0x01, lhs.b);
	EXPECT_EQ(0x10, lhs.a);
}

// Subtract one color from another.
TEST_F(ColorTests, SubtractColors) {
	const b2ParticleColor rhs(0x12, 0x13, 0x21, 0x01);
	const b2ParticleColor lhs(0x44, 0x33, 0x22, 0x11);
	b2ParticleColor result = lhs - rhs;
	EXPECT_EQ(0x32, result.r);
	EXPECT_EQ(0x20, result.g);
	EXPECT_EQ(0x01, result.b);
	EXPECT_EQ(0x10, result.a);
}

// Compare two colors.
TEST_F(ColorTests, Compare) {
	const b2ParticleColor violet(0x8d, 0x38, 0xc9, 0xff);
	const b2ParticleColor color(0x8d, 0x38, 0xc9, 0xff);
	EXPECT_TRUE(violet == color);
}

// Test color mixing with no mixing between colors.
TEST_F(ColorTests, MixNoMixing) {
	const b2ParticleColor colorAStart(0xff, 0x10, 0x44, 0xff);
	const b2ParticleColor colorBStart(0x01, 0x40, 0xff, 0x80);
	b2ParticleColor colorA = colorAStart;
	b2ParticleColor colorB = colorBStart;
	colorA.Mix(&colorB, 0);
	EXPECT_TRUE(colorA == colorAStart);
	EXPECT_TRUE(colorB == colorBStart);
}

// Test color mixing with maximum mixing between colors.
TEST_F(ColorTests, MixFullMixing) {
	b2ParticleColor colorA(0xff, 0x10, 0x44, 0xff);
	b2ParticleColor colorB(0x01, 0x40, 0xff, 0x80);
	colorA.Mix(&colorB, 128);
	EXPECT_EQ(0x80, colorA.r);
	EXPECT_EQ(0x28, colorA.g);
	EXPECT_EQ(0xa1, colorA.b);
	EXPECT_EQ(0xbf, colorA.a);
	EXPECT_EQ(0x80, colorB.r);
	EXPECT_EQ(0x28, colorB.g);
	EXPECT_EQ(0xa2, colorB.b);
	EXPECT_EQ(0xc0, colorB.a);
}

// Test color mixing with full mixing using the static method.
TEST_F(ColorTests, MixColorsFullMixing) {
	b2ParticleColor colorA(0xff, 0x10, 0x44, 0xff);
	b2ParticleColor colorB(0x01, 0x40, 0xff, 0x80);
	b2ParticleColor::MixColors(&colorA, &colorB, 128);
	EXPECT_EQ(0x80, colorA.r);
	EXPECT_EQ(0x28, colorA.g);
	EXPECT_EQ(0xa1, colorA.b);
	EXPECT_EQ(0xbf, colorA.a);
	EXPECT_EQ(0x80, colorB.r);
	EXPECT_EQ(0x28, colorB.g);
	EXPECT_EQ(0xa2, colorB.b);
	EXPECT_EQ(0xc0, colorB.a);
}

int main(int argc, char **argv)
{
	::testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
