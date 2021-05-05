#ifndef _POLYGON_H
#define _POLYGON_H
#include <cassert>
#include <iostream>

#include "vec3.hpp"

struct Polygon {
  unsigned int nVertices;  // 頂点数
  float* vertices;         // 頂点座標の配列
  unsigned int* indices;   // verticesへのインデックス配列
  float* normals;          // 頂点ごとの法線の配列
  float* uvs;              // 頂点ごとのUV座標の配列
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

  // 指定した頂点座標の位置の頂点座標をVec3で取得する
  Vec3 getVertex(unsigned int vertexIdx) const {
    assert(vertexIdx <= nVertices);
    return Vec3(vertices[3 * vertexIdx], vertices[3 * vertexIdx + 1],
                vertices[3 * vertexIdx + 2]);
  }

  // 指定した面の頂点座標配列へのインデックスを取得する
  std::array<unsigned int, 3> getIndices(unsigned int faceIdx) const {
    assert(faceIdx <= nFaces());
    return {indices[3 * faceIdx + 0], indices[3 * faceIdx + 1],
            indices[3 * faceIdx + 2]};
  }

  // 指定した頂点座標の位置の法線をVec3で取得する
  Vec3 getNormal(unsigned int vertexIdx) const {
    assert(vertexIdx <= nVertices);
    return Vec3(normals[3 * vertexIdx], normals[3 * vertexIdx + 1],
                normals[3 * vertexIdx + 2]);
  }

  // 指定した頂点座標の位置のUV座標を取得する
  std::pair<float, float> getUV(unsigned int vertexIdx) const {
    assert(vertexIdx <= nVertices);
    return {uvs[2 * vertexIdx], uvs[2 * vertexIdx + 1]};
  }

  // 頂点ごとの法線が存在するか
  bool hasNormals() const { return normals != nullptr; }
  // 頂点ごとのUVが存在するか
  bool hasUVs() const { return uvs != nullptr; }

  // 面の数を返す
  unsigned int nFaces() const { return nVertices / 3; };
};

#endif