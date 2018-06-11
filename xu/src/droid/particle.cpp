#include "droid/particle.h"

using namespace droid;

Particle::Particle(double _x, double _y, double _o, double _weight): x(_x), y(_y), o(_o), weight(_weight) {
}

Particle::Particle(const Particle& ref): x(ref.x), y(ref.y), o(ref.o), weight(ref.weight) {
}

Particle& Particle::set(double _x, double _y, double _o, double _weight) {
  this->x = _x;
  this->y = _y;
  this->o = _o;
  this->weight = _weight;
  return *this;
}

bool Particle::equals(double _x, double _y, double _o, double _weight) const {
  return this->x == _x && this->y == _y && this->o == _o && this->weight == _weight;
}
