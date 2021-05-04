#ifndef _TRIANGLE_H
#define _TRIANGLE_H
#include "aabb.hpp"
#include "intersect-info.hpp"
#include "polygon.hpp"

class Triangle {
 private:
  const Polygon* polygon;
  const unsigned int faceID;

 public:
  Triangle(const Polygon* polygon, unsigned int faceID)
      : polygon(polygon), faceID(faceID) {}

  AABB calcAABB() const {
    const auto indices = polygon->getIndices(faceID);
    const Vec3 v1 = polygon->getVertex(indices[0]);
    const Vec3 v2 = polygon->getVertex(indices[1]);
    const Vec3 v3 = polygon->getVertex(indices[2]);

    Vec3 pMin, pMax;
    for (int i = 0; i < 3; ++i) {
      pMin[i] = std::min(std::min(v1[i], v2[i]), v3[i]);
      pMax[i] = std::max(std::max(v1[i], v2[i]), v3[i]);
    }

    return AABB(pMin, pMax);
  }

  bool intersect(const Ray& ray, IntersectInfo& info) const { return false; }
};

#endif