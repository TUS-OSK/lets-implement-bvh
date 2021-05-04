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

  bool intersect(const Ray& ray, IntersectInfo& info) const {
    const auto indices = polygon->getIndices(faceID);
    const Vec3 v1 = polygon->getVertex(indices[0]);
    const Vec3 v2 = polygon->getVertex(indices[1]);
    const Vec3 v3 = polygon->getVertex(indices[2]);

    // https://www.tandfonline.com/doi/abs/10.1080/10867651.1997.10487468
    constexpr float EPS = 1e-8;
    const Vec3 e1 = v2 - v1;
    const Vec3 e2 = v3 - v1;

    const Vec3 pvec = cross(ray.direction, e2);
    const float det = dot(e1, pvec);

    if (det > -EPS && det < EPS) return false;
    const float invDet = 1.0f / det;

    const Vec3 tvec = ray.origin - v1;
    const float u = dot(tvec, pvec) * invDet;
    if (u < 0.0f || u > 1.0f) return false;

    const Vec3 qvec = tvec - v1;
    const float v = dot(ray.direction, qvec) * invDet;
    if (v < 0.0f || u + v > 1.0f) return false;

    const float t = dot(e2, qvec) * invDet;
    if (t < ray.tmin && t > ray.tmax) return false;

    info.t = t;
    info.hitPos = ray(t);
    info.barycentric[0] = u;
    info.barycentric[1] = v;

    return true;
  }
};

#endif