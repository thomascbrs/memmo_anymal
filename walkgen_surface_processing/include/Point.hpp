#ifndef _POINT_H
#define _POINT_H

#include "common.hpp"

class Point {
 public:
  double x, y;

  Point();
  Point(std::ifstream &fin);
  Point(double x, double y);

  friend std::ostream &operator<<(std::ostream &os, const Point &p);
  friend Point operator+(const Point &a, const Point &b);
  friend double area(const Point &a, const Point &b, const Point &c);
  friend bool left(const Point &a, const Point &b, const Point &c);
  friend bool leftOn(const Point &a, const Point &b, const Point &c);
  friend bool right(const Point &a, const Point &b, const Point &c);
  friend bool rightOn(const Point &a, const Point &b, const Point &c);
  friend bool collinear(const Point &a, const Point &b, const Point &c);
  friend double sqdist(const Point &a, const Point &b);

  // Operator ==, necessary to register a list
  bool operator==(const Point &other) const { return x == other.x && y == other.y; }
};

#endif
