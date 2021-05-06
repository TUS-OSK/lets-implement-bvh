#ifndef _CAMERA_H
#define _CAMERA_H
#include "core/ray.hpp"
#include "core/vec3.hpp"

class Camera {
 private:
  Vec3 camPos;
  Vec3 camForward;
  Vec3 camRight;
  Vec3 camUp;

 public:
  Camera(const Vec3& camPos, const Vec3& camForward)
      : camPos(camPos), camForward(camForward) {
    camRight = normalize(cross(camForward, Vec3(0, 1, 0)));
    camUp = normalize(cross(camRight, camForward));

    std::cout << "camPos: " << camPos << std::endl;
    std::cout << "camForward: " << camForward << std::endl;
    std::cout << "camRight: " << camRight << std::endl;
    std::cout << "camUp: " << camUp << std::endl;
  }

  Ray sampleRay(float u, float v) const {
    const Vec3 pinholePos = camPos + camForward;
    const Vec3 sensorPos = camPos + u * camRight + v * camUp;
    return Ray(camPos, normalize(pinholePos - sensorPos));
  }
};

#endif