#include "Point.hpp"

Point::Point() : x(0), y(0) {}

Point::Point(double x, double y) : x(x), y(y) {}

Point operator+(const Point &a, const Point &b) { return Point(a.x + b.x, b.y + b.y); }

Point::Point(std::ifstream &fin) {
  char c;
  fin >> c >> x >> c >> y >> c;
}

std::ostream &operator<<(std::ostream &os, const Point &p) { return os << "(" << p.x << ", " << p.y << ")"; }

double area(const Point &a, const Point &b, const Point &c) {
  return (((b.x - a.x) * (c.y - a.y)) - ((c.x - a.x) * (b.y - a.y)));
}

bool left(const Point &a, const Point &b, const Point &c) { return area(a, b, c) > 0; }

bool leftOn(const Point &a, const Point &b, const Point &c) { return area(a, b, c) >= 0; }

bool right(const Point &a, const Point &b, const Point &c) { return area(a, b, c) < 0; }

bool rightOn(const Point &a, const Point &b, const Point &c) { return area(a, b, c) <= 0; }

bool collinear(const Point &a, const Point &b, const Point &c) { return area(a, b, c) == 0; }

double sqdist(const Point &a, const Point &b) {
  double dx = b.x - a.x;
  double dy = b.y - a.y;
  return dx * dx + dy * dy;
}
