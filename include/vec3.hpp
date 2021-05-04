#ifndef _VEC3_H
#define _VEC3_H

#include <array>
#include <cmath>
#include <iostream>

class Vec3 {
 private:
  std::array<float, 3> v;

 public:
  Vec3() : v{0, 0, 0} {}
  Vec3(float value) : v{value, value, value} {}
  Vec3(float x, float y, float z) : v{x, y, z} {}

  float operator[](int i) const {
#ifndef NDEBUG
    return v.at(i);
#else
    return v[i];
#endif
  }
  float& operator[](int i) {
#ifndef NDEBUG
    return v.at(i);
#else
    return v[i];
#endif
  }

  bool operator==(const Vec3& rhs) const {
    return v[0] == rhs[0] && v[1] == rhs[1] && v[2] == rhs[2];
  }
};

inline Vec3 operator+(const Vec3& v1, const Vec3& v2) {
  Vec3 ret;
  for (int i = 0; i < 3; ++i) {
    ret[i] = v1[i] + v2[i];
  }
  return ret;
}
inline Vec3 operator-(const Vec3& v1, const Vec3& v2) {
  Vec3 ret;
  for (int i = 0; i < 3; ++i) {
    ret[i] = v1[i] - v2[i];
  }
  return ret;
}

inline Vec3 operator*(const Vec3& v, float k) {
  Vec3 ret;
  for (int i = 0; i < 3; ++i) {
    ret[i] = v[i] * k;
  }
  return ret;
}
inline Vec3 operator*(float k, const Vec3& v) { return v * k; }

inline Vec3 operator/(const Vec3& v, float k) {
  Vec3 ret;
  for (int i = 0; i < 3; ++i) {
    ret[i] = v[i] / k;
  }
  return ret;
}
inline Vec3 operator/(float k, const Vec3& v) {
  Vec3 ret;
  for (int i = 0; i < 3; ++i) {
    ret[i] = k / v[i];
  }
  return ret;
}

inline float length(const Vec3& v) {
  return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
}
inline float length2(const Vec3& v) {
  return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
}

inline float dot(const Vec3& v1, const Vec3& v2) {
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

inline Vec3 cross(const Vec3& v1, const Vec3& v2) {
  return Vec3(v1[1] * v2[2] - v1[2] * v2[1], v1[2] * v2[0] - v1[0] * v2[2],
              v1[0] * v2[1] - v1[1] * v2[0]);
}

inline Vec3 normalize(const Vec3& v) { return v / length(v); }

inline std::ostream& operator<<(std::ostream& stream, const Vec3& v) {
  stream << "(" << v[0] << ", " << v[1] << ", " << v[2] << ")";
  return stream;
}

#endif