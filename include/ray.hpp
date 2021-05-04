#ifndef _RAY_H
#define _RAY_H
#include <limits>

#include "vec3.hpp"

struct Ray {
  Vec3 origin;
  Vec3 direction;
  Vec3 dirInv;
  int dirInvSign[3];
  mutable float tmin{1e-3f};
  mutable float tmax{std::numeric_limits<float>::max()};

  explicit Ray(const Vec3& origin, const Vec3& direction)
      : origin(origin), direction(direction), dirInv(1.0f / direction) {
    for (int i = 0; i < 3; ++i) {
      dirInvSign[i] = dirInv[i] > 0 ? 0 : 1;
    }
  }

  Vec3 operator()(float t) const { return origin + t * direction; }
};

#endif