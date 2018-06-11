#include "droid/movement.h"

using namespace droid;

Movement::Movement(int _x, int _y, int _cost): x(_x), y(_y), cost(_cost) {
}

Movement::Movement(const Movement& ref): x(ref.x), y(ref.y), cost(ref.cost) {
}

Movement& Movement::set(int _x, int _y, int _cost) {
	this->x = _x;
	this->y = _y;
	this->cost = _cost;
	return *this;
}

bool Movement::equals(int _x, int _y, int _cost) const {
	return this->x == _x && this->y == _y && this->cost == _cost;
}
