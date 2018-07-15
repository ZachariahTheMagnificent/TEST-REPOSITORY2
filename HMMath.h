#pragma once

#define _USE_MATH_DEFINES
#define CIRCLE 0
#define RECT 1
#define PLAYER_RADIUS 50.f
#define MIN_FACING_DP 0.70710678118 // cos(45 * PI / 180)

#include <math.h>

struct Rotator {
	float radians;
	float notches;

	static constexpr float facingTolerance = 45 * M_PI / 180;
	static constexpr float notchestoRadians = 2 * M_PI / 1024;
	static constexpr float radiansToNotches = 1024 / (2 * M_PI);

	Rotator(int inNotches) :
		notches(inNotches) {
		radians = notches * notchestoRadians;
	}

	struct Normal ToVector();

	Rotator operator - (const Rotator& other) const {
		float notchOffset = other.notches - notches;
		float radianOffset = notchOffset * notchestoRadians;
		Rotator offsetRotator(radianOffset);
		offsetRotator.notches = notchOffset;
		return offsetRotator;
	}

	bool operator < (const int other) const {
		return notches < other;
		// TODO: consider normalizing
	}

	bool operator < (const Rotator& other) const {
		return notches < other.notches;
		// TODO: consider normalizing
	}
};

struct Vec2 {
	int x = 0, y = 0;

	Vec2() = default;
	Vec2(const Vec2& other) {
		x = other.x;
		y = other.y;
	}

	Vec2(int inX, int inY) :
		x(inX),
		y(inY)
	{}

	bool operator < (const Vec2& other) const;
	bool operator >= (const Vec2& other) const;
	Vec2 operator - (const Vec2& other) const;
	void operator -= (const Vec2& other);
	Vec2 operator + (const Vec2& other) const;
	void operator += (const Vec2& other);

	float SizeSquared() const;

	float Size() const;
};

struct Normal {
	float x = 0.f, y = 0.f;

	Normal() = delete;
	Normal(const Normal& other) {
		*this = other;
	}
	Normal(const float inX, const float inY, bool bNormalize = true);
	Normal(const Vec2 vector, bool bNormalize = true);
	void Normalize();
	void Normalize(const float inX, const float inY);
	float Dot(Normal& other) const;
	float OuterProduct(const Normal& other) const;
	Rotator ToRotator();
	void Swizzle() {
		float temp = x;
		x = y;
		y = temp;
	}
};