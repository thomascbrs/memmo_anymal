// Written by Mark Bayazit (darkzerox)
// March 23, 2009

// #include <vector>
// #include <stdlib.h>
// #include <stdio.h>
// #include <algorithm>
#include <limits>
#include "Bayazit.hpp"

Bayazit::Bayazit(){};

bool Bayazit::isReflex(const Polygon &poly, const int &i) {
  return right(at(poly, i - 1), at(poly, i), at(poly, i + 1));
}

Point Bayazit::intersection(const Point &p1, const Point &p2, const Point &q1, const Point &q2) {
  Point i = Point();
  double a1, b1, c1, a2, b2, c2, det;
  a1 = p2.y - p1.y;
  b1 = p1.x - p2.x;
  c1 = a1 * p1.x + b1 * p1.y;
  a2 = q2.y - q1.y;
  b2 = q1.x - q2.x;
  c2 = a2 * q1.x + b2 * q1.y;
  det = a1 * b2 - a2 * b1;
  if (!eq(det, 0)) {  // lines are not parallel
    i.x = (b2 * c1 - b1 * c2) / det;
    i.y = (a1 * c2 - a2 * c1) / det;
  }
  return i;
}

void Bayazit::swap(int &a, int &b) {
  int c;
  c = a;
  a = b;
  b = c;
}

std::vector<std::vector<std::shared_ptr<Point>>> Bayazit::decomposePolyWrapper(Polygon poly) {
  polys_.clear();
  steinerPoints_.clear();
  reflexVertices_.clear();
  decomposePoly(poly);
  // std::cout << "DECOMPO WORKS" << std::endl;
  std::vector<std::vector<std::shared_ptr<Point>>> polys_return;
  for (auto &polygon : polys_) {
    std::vector<std::shared_ptr<Point>> polygon_tmp;
    for (auto &point : polygon) {
      polygon_tmp.push_back(std::make_shared<Point>(point));
    }
    polys_return.push_back(polygon_tmp);
  }
  return polys_return;
}

void Bayazit::decomposePoly(Polygon poly) {
  Point upperInt = Point();
  Point lowerInt = Point();
  Point p = Point();
  Point closestVert = Point();
  double upperDist = 0.0, lowerDist = 0.0, closestDist = 0.0, d = 0.0;
  int upperIndex = 0, lowerIndex = 0, closestIndex = 0;
  Polygon lowerPoly, upperPoly;
  lowerPoly.clear();
  upperPoly.clear();

  for (int i = 0; i < static_cast<int>(poly.size()); ++i) {
    if (isReflex(poly, i)) {
      reflexVertices_.push_back(poly[static_cast<size_t>(i)]);
      upperDist = lowerDist = std::numeric_limits<double>::max();
      for (int j = 0; j < static_cast<int>(poly.size()); ++j) {
        if (left(at(poly, i - 1), at(poly, i), at(poly, j)) &&
            rightOn(at(poly, i - 1), at(poly, i), at(poly, j - 1))) {  // if line intersects with an edge
          p = intersection(at(poly, i - 1), at(poly, i), at(poly, j),
                           at(poly, j - 1));             // find the point of intersection
          if (right(at(poly, i + 1), at(poly, i), p)) {  // make sure it's inside the poly
            d = sqdist(poly[static_cast<size_t>(i)], p);
            if (d < lowerDist) {  // keep only the closest intersection
              lowerDist = d;
              lowerInt = p;
              lowerIndex = j;
            }
          }
        }
        if (left(at(poly, i + 1), at(poly, i), at(poly, j + 1)) &&
            rightOn(at(poly, i + 1), at(poly, i), at(poly, j))) {
          p = intersection(at(poly, i + 1), at(poly, i), at(poly, j), at(poly, j + 1));
          if (left(at(poly, i - 1), at(poly, i), p)) {
            d = sqdist(poly[static_cast<size_t>(i)], p);
            if (d < upperDist) {
              upperDist = d;
              upperInt = p;
              upperIndex = j;
            }
          }
        }
      }

      // if there are no vertices to connect to, choose a point in the middle
      if (lowerIndex == (upperIndex + 1) % static_cast<int>(poly.size())) {
        // printf("Case 1: Vertex(%d), lowerIndex(%d), upperIndex(%d), poly.size(%d)\n", i, lowerIndex, upperIndex,
        // (int) poly.size());
        p.x = (lowerInt.x + upperInt.x) / 2;
        p.y = (lowerInt.y + upperInt.y) / 2;
        steinerPoints_.push_back(p);

        if (i < upperIndex) {
          lowerPoly.insert(lowerPoly.end(), poly.begin() + i, poly.begin() + upperIndex + 1);
          lowerPoly.push_back(p);
          upperPoly.push_back(p);
          if (lowerIndex != 0) upperPoly.insert(upperPoly.end(), poly.begin() + lowerIndex, poly.end());
          upperPoly.insert(upperPoly.end(), poly.begin(), poly.begin() + i + 1);
        } else {
          if (i != 0) lowerPoly.insert(lowerPoly.end(), poly.begin() + i, poly.end());
          lowerPoly.insert(lowerPoly.end(), poly.begin(), poly.begin() + upperIndex + 1);
          lowerPoly.push_back(p);
          upperPoly.push_back(p);
          upperPoly.insert(upperPoly.end(), poly.begin() + lowerIndex, poly.begin() + i + 1);
        }
      } else {
        // connect to the closest point within the triangle
        // printf("Case 2: Vertex(%d), closestIndex(%d), poly.size(%d)\n", i, closestIndex, (int) poly.size());

        if (lowerIndex > upperIndex) {
          upperIndex += poly.size();
        }
        closestDist = std::numeric_limits<double>::max();
        for (int j = lowerIndex; j <= upperIndex; ++j) {
          if (leftOn(at(poly, i - 1), at(poly, i), at(poly, j)) &&
              rightOn(at(poly, i + 1), at(poly, i), at(poly, j))) {
            d = sqdist(at(poly, i), at(poly, j));
            if (d < closestDist) {
              closestDist = d;
              closestVert = at(poly, j);
              closestIndex = j % static_cast<int>(poly.size());
            }
          }
        }

        if (i < closestIndex) {
          lowerPoly.insert(lowerPoly.end(), poly.begin() + i, poly.begin() + closestIndex + 1);
          if (closestIndex != 0) upperPoly.insert(upperPoly.end(), poly.begin() + closestIndex, poly.end());
          upperPoly.insert(upperPoly.end(), poly.begin(), poly.begin() + i + 1);
        } else {
          if (i != 0) lowerPoly.insert(lowerPoly.end(), poly.begin() + i, poly.end());
          lowerPoly.insert(lowerPoly.end(), poly.begin(), poly.begin() + closestIndex + 1);
          upperPoly.insert(upperPoly.end(), poly.begin() + closestIndex, poly.begin() + i + 1);
        }
      }

      // solve smallest poly first
      if (lowerPoly.size() < upperPoly.size()) {
        decomposePoly(lowerPoly);
        decomposePoly(upperPoly);
      } else {
        decomposePoly(upperPoly);
        decomposePoly(lowerPoly);
      }
      return;
    }
  }
  polys_.push_back(poly);
}
