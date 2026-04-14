#include "Point.h"
#include <math.h>

Point::Point(float x, float y) {
	this->x = x;
	this->y = y;
}

void Point::Transform(float dx, float dy, float a) {
	float x = this->x;
	float y = this->y;

	this->x = x * cos(a) - y * sin(a) + dx;
	this->y = x * sin(a) + y * cos(a) + dy;
}

Point Point::Transformed(float dx, float dy, float a) {
    float x = this->x;
    float y = this->y;

    return Point(x * cos(a) - y * sin(a) + dx, x * sin(a) + y * cos(a) + dy);
}

float Point::DistanceSq(Point& b) {
	return (this->x - b.x) * (this->x - b.x) + (this->y - b.y) * (this->y - b.y);
}
float Point::DistanceSq(const Point& b) {
	return (this->x - b.x) * (this->x - b.x) + (this->y - b.y) * (this->y - b.y);
}
float Point::DistanceSq(Point* b) {
    return (this->x - b->x) * (this->x - b->x) + (this->y - b->y) * (this->y - b->y);
}
float Point::Length() {
    return sqrt(this->x * this->x + this->y * this->y);
}


LidarPoint::LidarPoint(float distance, float angle, uint8_t confidance) {
	this->distance = distance;
	this->angle = angle;
	this->confidance = confidance;
	this->x = distance * cos(angle);
	this->y = distance * sin(angle);
}