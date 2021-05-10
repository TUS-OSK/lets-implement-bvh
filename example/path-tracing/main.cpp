#define TINYOBJLOADER_IMPLEMENTATION
#include <omp.h>

#include <chrono>
#include <memory>
#include <string>

#include "bvh.hpp"
#include "camera.hpp"
#include "image.hpp"
#include "rng.hpp"
#include "tiny_obj_loader.h"

constexpr float PI = 3.14159265359f;
constexpr float INV_PI = 1.0f / PI;

Vec3 localToWorld(const Vec3& v, const Vec3& lx, const Vec3& ly,
                  const Vec3& lz) {
  return Vec3(v[0] * lx[0] + v[1] * ly[0] + v[2] * lz[0],
              v[0] * lx[1] + v[1] * ly[1] + v[2] * lz[1],
              v[0] * lx[2] + v[1] * ly[2] + v[2] * lz[2]);
}

void tangentSpaceBasis(const Vec3& n, Vec3& t, Vec3& b) {
  if (std::abs(n[1]) < 0.9f) {
    t = normalize(cross(n, Vec3(0, 1, 0)));
  } else {
    t = normalize(cross(n, Vec3(0, 0, -1)));
  }
  b = normalize(cross(t, n));
}

Vec3 sampleCosineHemisphere(float u, float v, float& pdf) {
  const float theta =
      0.5f * std::acos(std::clamp(1.0f - 2.0f * u, -1.0f, 1.0f));
  const float phi = 2.0f * PI * v;

  const float cosTheta = std::cos(theta);
  pdf = cosTheta * INV_PI;
  return Vec3(std::cos(phi) * std::sin(theta), cosTheta,
              std::sin(phi) * std::sin(theta));
}

Vec3 pathTracing(const Ray& ray_in, const OptimizedBVH& scene, RNG& rng) {
  constexpr int maxDepth = 100;
  const Vec3 rho{0.9f, 0.9f, 0.9f};

  Vec3 radiance{0, 0, 0};
  Vec3 throughput{1, 1, 1};
  Ray ray = ray_in;
  for (int i = 0; i < maxDepth; ++i) {
    const float russianRouletteProb =
        std::max(std::max(throughput[0], throughput[1]), throughput[2]);
    if (rng.getNext() > russianRouletteProb) {
      break;
    }
    throughput /= russianRouletteProb;

    IntersectInfo info;
    if (!scene.intersect(ray, info)) {
      radiance += throughput * Vec3(1);
      break;
    }

    if (dot(-ray.direction, info.hitNormal) < 0) {
      info.hitNormal = -info.hitNormal;
    }

    Vec3 t, b;
    tangentSpaceBasis(info.hitNormal, t, b);
    float pdf;
    const Vec3 directionTangent =
        sampleCosineHemisphere(rng.getNext(), rng.getNext(), pdf);
    const Vec3 direction = localToWorld(directionTangent, t, info.hitNormal, b);

    const Vec3 brdf = rho * INV_PI;
    const float cos = std::max(dot(direction, info.hitNormal), 0.0f);

    throughput *= brdf * cos / pdf;

    ray = Ray(info.hitPos, direction);
  }

  return radiance;
}

bool loadObj(const std::string& filename, std::vector<float>& vertices,
             std::vector<unsigned int>& indices, std::vector<float>& normals,
             std::vector<float>& uvs) {
  tinyobj::ObjReader reader;

  if (!reader.ParseFromFile(filename)) {
    if (!reader.Error().empty()) {
      std::cerr << reader.Error();
    }
    return false;
  }

  if (!reader.Warning().empty()) {
    std::cout << reader.Warning();
  }

  const auto& attrib = reader.GetAttrib();
  const auto& shapes = reader.GetShapes();

  vertices = attrib.vertices;
  if (attrib.normals.size() == attrib.vertices.size()) {
    normals = attrib.normals;
  }
  if (attrib.texcoords.size() == (attrib.vertices.size() / 3) * 2) {
    uvs = attrib.texcoords;
  }

  for (size_t s = 0; s < shapes.size(); ++s) {
    for (const auto& idx : shapes[s].mesh.indices) {
      indices.push_back(idx.vertex_index);
    }
  }

  return true;
}

int main() {
  const std::string filename = "sponza.obj";
  const int width = 512;
  const int height = 512;
  const int samples = 1;
  const Vec3 camPos(-10, 7, 0);
  const Vec3 camForward(1, 0, 0);

  std::vector<float> vertices;
  std::vector<unsigned int> indices;
  std::vector<float> normals;
  std::vector<float> uvs;

  if (!loadObj(filename, vertices, indices, normals, uvs)) {
    std::exit(EXIT_FAILURE);
  }

  const auto polygon =
      std::make_shared<Polygon>(indices.size(), vertices.data(), indices.data(),
                                normals.data(), uvs.data());
  std::cout << "vertices: " << polygon->nVertices << std::endl;
  std::cout << "faces: " << polygon->nFaces() << std::endl;

  OptimizedBVH bvh(*polygon);
  bvh.buildBVH();
  std::cout << "nodes: " << bvh.nNodes() << std::endl;
  std::cout << "internal nodes: " << bvh.nInternalNodes() << std::endl;
  std::cout << "leaf nodes: " << bvh.nLeafNodes() << std::endl;
  std::cout << "bbox: " << bvh.rootAABB() << std::endl;

  Image img(width, height);
  Camera camera(camPos, camForward);

  const auto startTime = std::chrono::system_clock::now();
#pragma omp parallel for schedule(dynamic, 1)
  for (int j = 0; j < height; ++j) {
    for (int i = 0; i < width; ++i) {
      RNG rng(i + width * j);

      Vec3 color{0, 0, 0};
      for (int k = 0; k < samples; ++k) {
        const float u = (2.0f * (i + rng.getNext()) - width) / height;
        const float v = (2.0f * (j + rng.getNext()) - height) / height;
        const Ray ray = camera.sampleRay(u, v);
        color += pathTracing(ray, bvh, rng);
      }
      color /= Vec3(samples);

      img.setPixel(i, j, color);
    }
  }
  std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now() - startTime)
                   .count()
            << "ms" << std::endl;

  img.writePPM("output.ppm");

  return 0;
}