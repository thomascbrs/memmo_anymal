#ifndef _BAYAZIT_H
#define _BAYAZIT_H

#include <Point.hpp>
#include <memory>

typedef std::vector<Point> Polygon;

class Bayazit {
 public:
  // Constructor
  Bayazit();

  // Destructor
  ~Bayazit() {}

  std::vector<std::vector<std::shared_ptr<Point>>> decomposePolyWrapper(Polygon poly);

 private:
  bool isReflex(const Polygon &poly, const int &i);
  Point intersection(const Point &p1, const Point &p2, const Point &q1, const Point &q2);
  void swap(int &a, int &b);
  void decomposePoly(Polygon poly);

  std::vector<Polygon> polys_ = {};
  Polygon steinerPoints_ = {};
  Polygon reflexVertices_ = {};
};

#endif
