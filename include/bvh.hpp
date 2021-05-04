#ifndef _BVH_H
#define _BVH_H
#include <vector>

#include "triangle.hpp"

class BVH {
 private:
  std::vector<Triangle> primitives;

 public:
  BVH(const Polygon& polygon) {
    // PolygonからTriangleを抜き出して追加していく
    for (int f = 0; f < polygon.nFaces(); ++f) {
      primitives.emplace_back(&polygon, f);
    }
  }

  bool buildBVH() const { return false; }
  bool intersect(const Ray& ray, IntersectInfo& info) const { return false; }
};

#endif