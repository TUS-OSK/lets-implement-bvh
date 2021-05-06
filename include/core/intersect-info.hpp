#ifndef _INTERSECT_INFO_H
#define _INTERSECT_INFO_H
#include "core/vec3.hpp"

struct IntersectInfo {
  float t;
  Vec3 hitPos;
  Vec3 hitNormal;
  float barycentric[2];
  float uv[2];
  int geomID;
  int primID;
};

#endif