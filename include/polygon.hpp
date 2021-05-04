#ifndef _POLYGON_H
#define _POLYGON_H
#include <iostream>

#include "vec3.hpp"

struct Polygon {
  unsigned int nVertices;  // 頂点数
  float* vertices;         // 頂点座標の配列
  unsigned int* indices;   // 頂点配列
  float* normals;
  float* uvs;
  int* geomIDs;

  Polygon(unsigned int nVertices, float* vertices, unsigned int* indices,
          float* normals = nullptr, float* uvs = nullptr,
          int* geomIDs = nullptr)
      : nVertices(nVertices),
        vertices(vertices),
        indices(indices),
        normals(normals),
        uvs(uvs),
        geomIDs(geomIDs) {}

  Vec3 getVertex(int vertexIdx) const {
    if (vertexIdx > nVertices) {
      std::cerr << "vertex index is out of range" << std::endl;
      std::exit(EXIT_FAILURE);
    }
    return Vec3(vertices[3 * vertexIdx], vertices[3 * vertexIdx + 1],
                vertices[3 * vertexIdx + 2]);
  }

  std::array<unsigned int, 3> getIndices(int faceIdx) const {
    if (faceIdx > nFaces()) {
      std::cerr << "face index is out of range" << std::endl;
      std::exit(EXIT_FAILURE);
    }
    return {indices[3 * faceIdx + 0], indices[3 * faceIdx + 1],
            indices[3 * faceIdx + 2]};
  }

  unsigned int nFaces() const { return nVertices / 3; };
};

#endif