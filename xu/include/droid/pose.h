#ifndef DROID_POSE_H
#define DROID_POSE_H

namespace droid {

struct Pose {
  explicit Pose(double _x = 0.0, double _y = 0.0, double _o = 0.0);
  Pose(const Pose&);

  Pose& set(double, double, double);
  inline Pose& set(const Pose& ref) {
    return this->set(ref.x, ref.y, ref.o);
  }

  bool equals(double, double, double) const;
  inline bool equals(const Pose& ref) const {
    return this->equals(ref.x, ref.y, ref.o);
  }

  inline bool operator ==(const Pose& ref) const {
    return this->equals(ref.x, ref.y, ref.o);
  }
  inline bool operator !=(const Pose& ref) const {
    return !this->equals(ref.x, ref.y, ref.o);
  }

  inline Pose& operator =(const Pose& ref) {
    return this->set(ref.x, ref.y, ref.o);
  }

  double x;
  double y;
  double o;
};

};

#endif
