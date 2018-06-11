#ifndef DROID_GRID_H
#define DROID_GRID_H

namespace droid {

struct Grid{
  explicit Grid(int _known = 0, int _occupied = 0, int _closed = 0, int _expand = 0, int _heuristic = 0);
  Grid(const Grid&);

  Grid& set(int _known, int _occupied, int _closed, int _expand, int _heuristic);
  inline Grid& set(const Grid& ref) {
    return this->set(ref.known, ref.occupied, ref.closed, ref.expand, ref.heuristic);
  }

  bool equals(int _known, int _occupied, int _closed, int _expand, int _heuristic) const;
  inline bool equals(const Grid& ref) const {
    return this->equals(ref.known, ref.occupied, ref.closed, ref.expand, ref.heuristic);
  }

  inline Grid& operator =(const Grid& ref) {
    return this->set(ref.known, ref.occupied, ref.closed, ref.expand, ref.heuristic);
  }

  inline bool operator ==(const Grid& ref) const {
    return this->equals(ref.known, ref.occupied, ref.closed, ref.expand, ref.heuristic);
  }
  inline bool operator !=(const Grid& ref) const {
    return !this->equals(ref.known, ref.occupied, ref.closed, ref.expand, ref.heuristic);
  }

  int known;
  int occupied;
  int closed;
  int expand;
  int heuristic;
};

};

#endif
