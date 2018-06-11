#include "droid/grid.h"

using namespace droid;

Grid::Grid(int _known, int _occupied, int _closed, int _expand, int _heuristic): known(_known), occupied(_occupied), closed(_closed), expand(_expand), heuristic(_heuristic) {
}

Grid::Grid(const Grid& ref): known(ref.known), occupied(ref.occupied), closed(ref.closed), expand(ref.expand), heuristic(ref.heuristic) {
}

Grid& Grid::set(int _known, int _occupied, int _closed, int _expand, int _heuristic) {
	this->known = _known;
	this->occupied = _occupied;
	this->closed = _closed;
	this->expand = _expand;
	this->heuristic = _heuristic;
	return *this;
}

bool Grid::equals(int _known, int _occupied, int _closed, int _expand, int _heuristic) const {
	return this->known == _known && this->occupied == _occupied && this->closed == _closed && this->expand == _expand && this->heuristic == _heuristic;
}
