#include "HMMath.h"

// Rotator struct
constexpr float Rotator::facingTolerance;
constexpr float Rotator::notchestoRadians;
constexpr float Rotator::radiansToNotches;

Normal Rotator::ToVector() {
	float xComp = cos(radians);
	float yComp = sin(radians);
	Normal result(xComp, yComp, false);
	return result;
}

// Vec2 struct (lower precision for large distances)
bool Vec2::operator < (const Vec2& other) const {
	return x < other.x && y < other.y;
}

bool Vec2::operator >= (const Vec2& other) const {
	return x >= other.x && y >= other.y;
}

Vec2 Vec2::operator - (const Vec2& other) const {
	return Vec2(x - other.x, y - other.y);
}

void Vec2::operator -= (const Vec2& other) {
	x -= other.x;
	y -= other.y;
}

Vec2 Vec2::operator + (const Vec2& other) const {
	return Vec2(x + other.x, y + other.y);
}

void Vec2::operator += (const Vec2& other) {
	x += other.x;
	y += other.y;
}

float Vec2::SizeSquared() const {
	return x * x + y * y;
}

float Vec2::Size() const {
	return sqrt(SizeSquared());
}

// Normal struct (used for high precision directional calculations)
Rotator Normal::ToRotator() {
	return Rotator(acos(x));
}

void Normal::Normalize() {
	const float magnitude = sqrt(x * x + y * y);
	const float coefficient = 1.f / magnitude;
	x *= coefficient;
	y *= coefficient;
}

void Normal::Normalize(const float inX, const float inY) {
	const float magnitude = sqrt(inX * inX + inY * inY);
	const float coefficient = 1.f / magnitude;
	x = inX * coefficient;
	y = inY * coefficient;
}

Normal::Normal(const float inX, const float inY, bool bNormalize) {
	if (bNormalize)
		Normalize(inX, inY);
	else {
		x = inX;
		y = inY;
	}
}

Normal::Normal(const Vec2 vector, bool bNormalize) {
	if (bNormalize)
		Normalize(vector.x, vector.y);
	else {
		x = vector.x;
		y = vector.y;
	}
}

float Normal::Dot(Normal& other) const {
	return x * other.x + y * other.y;
}

float Normal::OuterProduct(const Normal& other) const {
	return x * other.y - other.x * y;
}