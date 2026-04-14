#pragma once
#include <cstdint>
#include <chrono>


class Point {
public:
	float x;
	float y;
	Point(float x, float y);
	void Transform(float dx, float dy, float a);
	float DistanceSq(Point& b);
	float DistanceSq(const Point& b);
	float DistanceSq(Point* b);
	float Length();
	Point Transformed(float dx, float dy, float a);
};

class LidarPoint {
public:
	float x;
	float y;
	uint8_t confidance;
	float distance;
	float angle;
	std::chrono::_V2::high_resolution_clock::time_point ts;
	std::chrono::_V2::high_resolution_clock::time_point te;
	LidarPoint(float distance, float angle, uint8_t confidance);
};