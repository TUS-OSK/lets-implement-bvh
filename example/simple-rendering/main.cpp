#define TINYOBJLOADER_IMPLEMENTATION
#include <chrono>
#include <memory>
#include <string>

#include "bvh.hpp"
#include "camera.hpp"
#include "image.hpp"
#include "tiny_obj_loader.h"

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
  const std::string filename = "bunny.obj";
  const int width = 512;
  const int height = 512;
  const Vec3 camPos(0, 1, 2);
  const Vec3 camForward(0, 0, -1);

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
  for (int j = 0; j < height; ++j) {
    for (int i = 0; i < width; ++i) {
      const float u = (2.0f * i - width) / height;
      const float v = (2.0f * j - height) / height;
      const Ray ray = camera.sampleRay(u, v);

      IntersectInfo info;
      if (bvh.intersect(ray, info)) {
        img.setPixel(i, j, 0.5f * (info.hitNormal + Vec3(1.0f)));
      } else {
        img.setPixel(i, j, Vec3(0));
      }
    }
  }
  std::cout << std::chrono::duration_cast<std::chrono::milliseconds>(
                   std::chrono::system_clock::now() - startTime)
                   .count()
            << "ms" << std::endl;

  img.writePPM("output.ppm");

  return 0;
}