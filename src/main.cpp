#define TINYOBJLOADER_IMPLEMENTATION
#include <memory>
#include <string>

#include "bvh.hpp"
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
  std::string filename = "CornellBox-Original.obj";

  std::vector<float> vertices;
  std::vector<unsigned int> indices;
  std::vector<float> normals;
  std::vector<float> uvs;

  if (!loadObj(filename, vertices, indices, normals, uvs)) {
    std::exit(EXIT_FAILURE);
  }

  const auto polygon = std::make_shared<Polygon>(
      indices.size(), vertices.data(), indices.data());

  std::cout << "vertices: " << polygon->nVertices << std::endl;
  std::cout << "faces: " << polygon->nFaces() << std::endl;

  BVH bvh(*polygon);
  bvh.buildBVH();
  std::cout << "nodes: " << bvh.nNodes() << std::endl;
  std::cout << "internal nodes: " << bvh.nInternalNodes() << std::endl;
  std::cout << "leaf nodes: " << bvh.nLeafNodes() << std::endl;
  std::cout << "bbox: " << bvh.rootAABB() << std::endl;

  Ray ray(Vec3(0, 0, -10), Vec3(0, 0, 1));
  IntersectInfo info;
  if (bvh.intersect(ray, info)) {
    std::cout << "t: " << info.t << std::endl;
    std::cout << "hitPos: " << info.hitPos << std::endl;
    std::cout << "barycentric[0]: " << info.barycentric[0] << std::endl;
    std::cout << "barycentric[1]: " << info.barycentric[1] << std::endl;
  }

  return 0;
}