#ifndef DROID_NODE_H
#define DROID_NODE_H

#include <ostream>

namespace droid {

struct Node {
  explicit Node(int _x = 0, int _y = 0, int _cost = 0);
  Node(const Node&);

  Node& set(int _x, int _y, int _cost);
  inline Node& set(const Node& ref) {
    return this->set(ref.x, ref.y, ref.cost);
  }

  bool equals(int _x, int _y, int _cost) const;
  inline bool equals(const Node& ref) const {
    return this->equals(ref.x, ref.y, ref.cost);
  }

  inline bool operator ==(const Node& ref) const {
    return this->equals(ref.x, ref.y, ref.cost);
  }
  inline bool operator !=(const Node& ref) const {
    return !this->equals(ref.x, ref.y, ref.cost);
  }

  inline Node& operator =(const Node& ref) {
    return this->set(ref.x, ref.y, ref.cost);
  }

  int x;
  int y;
  int cost;
};

static std::ostream& operator <<(std::ostream&, const Node&);

};

#endif
