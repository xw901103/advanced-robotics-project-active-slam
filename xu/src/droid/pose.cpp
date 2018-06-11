#include "droid/pose.h"

using namespace droid;

Pose::Pose(double _x, double _y, double _o): x(_x), y(_y), o(_o) {
}

Pose::Pose(const Pose& ref): x(ref.x), y(ref.y), o(ref.o) {
}

Pose& Pose::set(double _x, double _y, double _o) {
  this->x = _x;
  this->y = _y;
  this->o = _o;
  return *this;
}

bool Pose::equals(double _x, double _y, double _o) const {
  return this->x == _x && this->y == _y && this->o == _o;
}
