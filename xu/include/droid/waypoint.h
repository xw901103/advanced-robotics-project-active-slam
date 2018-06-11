#ifndef DROID_WAYPOINT_H
#define DROID_WAYPOINT_H

#include <ostream>

namespace droid {

struct Waypoint{
  explicit Waypoint(double _x = 0.0, double _y = 0.0);
  Waypoint(const Waypoint&);

  Waypoint& set(double _x, double _y);
  inline Waypoint& set(const Waypoint& ref) {
    return this->set(ref.x, ref.y);
  }

  bool equals(double _x, double _y) const;
  inline bool equals(const Waypoint& ref) const {
    return this->equals(ref.x, ref.y);
  }

  inline bool operator ==(const Waypoint& ref) const {
    return this->equals(ref.x, ref.y);
  }
  inline bool operator !=(const Waypoint& ref) const {
    return !this->equals(ref.x, ref.y);
  }

  inline Waypoint& operator =(const Waypoint& ref) {
    return this->set(ref.x, ref.y);
  }

  double x;
  double y;
};

inline std::ostream& operator <<(std::ostream& _stream, const Waypoint& w) {
	_stream << "x: " << w.x << " y: " << w.y;
	return _stream;
}

inline Waypoint operator *(double v, const Waypoint& ref) {
	return Waypoint(ref.x * v, ref.y * v);
}

inline Waypoint operator +(const Waypoint& lhs, const Waypoint& rhs) {
	return Waypoint(lhs.x + rhs.x, lhs.y + rhs.y);
}

inline Waypoint operator -(const Waypoint& lhs, const Waypoint& rhs) {
	return Waypoint(lhs.x - rhs.x, lhs.y - rhs.y);
}

};

#endif
