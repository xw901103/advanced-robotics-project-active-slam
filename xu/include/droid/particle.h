#ifndef DROID_PARTICLE_H
#define DROID_PARTICLE_H

namespace droid {

struct Particle {
  explicit Particle(double _x = 0.0, double _y = 0.0, double _o = 0.0, double w = 0.0);
  Particle(const Particle&);

  Particle& set(double, double, double, double);
  inline Particle& set(const Particle& ref) {
    return this->set(ref.x, ref.y, ref.o, ref.weight);
  }

  bool equals(double, double, double, double) const;
  inline bool equals(const Particle& ref) const {
    return this->equals(ref.x, ref.y, ref.o, ref.weight);
  }

  inline bool operator ==(const Particle& ref) const {
    return this->equals(ref.x, ref.y, ref.o, ref.weight);
  }
  inline bool operator !=(const Particle& ref) const {
    return !this->equals(ref.x, ref.y, ref.o, ref.weight);
  }

  inline Particle& operator =(const Particle& ref) {
    return this->set(ref.x, ref.y, ref.o, ref.weight);
  }

  double x;
  double y;
  double o;
  double weight;
};

};

#endif
