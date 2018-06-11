#include "droid/waypoint.h"

using namespace droid;

Waypoint::Waypoint(double _x, double _y): x(_x), y(_y) {
}

Waypoint::Waypoint(const Waypoint& ref): x(ref.x), y(ref.y) {
}

Waypoint& Waypoint::set(double _x, double _y) {
	this->x = _x;
	this->y = _y;
	return *this;
}

bool Waypoint::equals(double _x, double _y) const {
	return this->x == _x && this->y == _y;
}
