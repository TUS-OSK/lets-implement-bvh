#define TINYOBJLOADER_IMPLEMENTATION
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
  const auto& materials = reader.GetMaterials();

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
  const std::string filename = "CornellBox-Original.obj";
  const int width = 512;
  const int height = 512;
  const Vec3 camPos(0, 0, 3);
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

  BVH bvh(*polygon);
  bvh.buildBVH();

  Image img(width, height);
  Camera camera(camPos, camForward);
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
  img.writePPM("output.ppm");

  return 0;
}