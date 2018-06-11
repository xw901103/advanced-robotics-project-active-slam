#ifndef DROID_MOVEMENT_H
#define DROID_MOVEMENT_H

namespace droid {

struct Movement {
  explicit Movement(int _x = 0, int _y = 0, int _cost = 0);
  Movement(const Movement&);

  Movement& set(int _x, int _y, int _cost);
  inline Movement& set(const Movement& ref) {
    return this->set(ref.x, ref.y, ref.cost);
  }

  bool equals(int _x, int _y, int _cost) const;
  inline bool equals(const Movement& ref) const {
    return this->equals(ref.x, ref.y, ref.cost);
  }

  inline bool operator ==(const Movement& ref) const {
    return this->equals(ref.x, ref.y, ref.cost);
  }
  inline bool operator !=(const Movement& ref) const {
    return !this->equals(ref.x, ref.y, ref.cost);
  }

  inline Movement& operator =(const Movement& ref) {
    return this->set(ref.x, ref.y, ref.cost);
  }

  int x;
  int y;
  int cost;
};

};

#endif
