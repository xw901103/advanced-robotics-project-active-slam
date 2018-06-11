#include "droid/node.h"

using namespace droid;

Node::Node(int _x, int _y, int _cost): x(_x), y(_y), cost(_cost) {
}

Node::Node(const Node& ref): x(ref.x), y(ref.y), cost(ref.cost) {
}

Node& Node::set(int _x, int _y, int _cost) {
	this->x = _x;
	this->y = _y;
	this->cost = _cost;
	return *this;
}

bool Node::equals(int _x, int _y, int _cost) const {
	return this->x == _x && this->y == _y && this->cost == _cost;
}

static std::ostream& operator<<(std::ostream& _stream, const Node& _node) {
	_stream << "x: " << _node.x << " y: " << _node.y;
	return _stream;
}
