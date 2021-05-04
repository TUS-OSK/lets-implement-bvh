#ifndef _BVH_H
#define _BVH_H
#include <numeric>
#include <vector>

#include "triangle.hpp"

class BVH {
 private:
  std::vector<Triangle> primitives;  // Primitive(三角形)の配列
  std::vector<AABB> bboxes;  // Primitiveのバウンディングボックスの配列
  std::vector<int> primIndices;  // primitivesへのインデックスの配列

  struct BVHNode {
    AABB bbox;             // バウンディングボックス
    int primitivesOffset;  // primIndicesへのオフセット
    int nPrimitives;       // ノードに含まれるPrimitiveの数
    int axis;              // 分割軸(traverseの最適化に使う)
    BVHNode* child[2];  // 子ノードへのポインタ, 両方nullptrだったら葉ノード
  };

  struct BVHStatistics {
    int nNodes{0};          // ノード総数
    int nInternalNodes{0};  // 中間ノードの数
    int nLeafNodes{0};      // 葉ノードの数
  };

  BVHNode* root;        // ルートノードへのポインタ
  BVHStatistics stats;  // BVHの統計情報

  // 再帰的にBVHのノードを構築していく
  BVHNode* buildBVHNode(int primStart, int primEnd,
                        std::vector<int>& primIndices) {
    // ノードの作成
    BVHNode* node = new BVHNode;

    // AABBの計算
    AABB bbox;
    for (int i = primStart; i < primEnd; ++i) {
      const int primIdx = primIndices[i];
      bbox = mergeAABB(bbox, bboxes[primIdx]);
    }

    const int nPrims = primEnd - primStart;
    if (nPrims < 4) {
      // 葉ノードの作成
      node->bbox = bbox;
      node->primitivesOffset = primStart;
      node->nPrimitives = nPrims;
      node->child[0] = nullptr;
      node->child[1] = nullptr;
      stats.nLeafNodes++;
      return node;
    }

    // 分割軸
    const int splitAxis = bbox.longestAxis();

    // 分割点
    const float splitPos = bbox.center()[splitAxis];

    // AABBの分割
    const int splitIdx =
        std::partition(primIndices.begin() + primStart,
                       primIndices.begin() + primEnd,
                       [&](int idx) {
                         return bboxes[idx].center()[splitAxis] < splitPos;
                       }) -
        primIndices.begin();

    // 分割が失敗した場合は葉ノードを作成
    if (splitIdx == primStart || splitIdx == primEnd) {
      node->bbox = bbox;
      node->primitivesOffset = primStart;
      node->nPrimitives = nPrims;
      node->child[0] = nullptr;
      node->child[1] = nullptr;
      stats.nLeafNodes++;
      return node;
    }

    // 中間ノードに情報をセット
    node->bbox = bbox;
    node->primitivesOffset = primStart;
    node->nPrimitives = nPrims;
    node->axis = splitAxis;

    // 左の子ノードで同様の計算
    node->child[0] = buildBVHNode(primStart, splitIdx, primIndices);
    // 右の子ノードで同様の計算
    node->child[1] = buildBVHNode(splitIdx, primEnd, primIndices);
    stats.nInternalNodes++;

    return node;
  }

  // 再帰的にBVHNodeを消去していく
  void deleteBVHNode(BVHNode* node) {
    if (node->child[0]) deleteBVHNode(node->child[0]);
    if (node->child[1]) deleteBVHNode(node->child[1]);
    delete node;
  }

  // 再帰的にBVHのtraverseを行う
  bool intersectNode(const BVHNode* node, Ray& ray, IntersectInfo& info) const {
    bool hit = false;

    // AABBとの交差判定
    if (node->bbox.intersect(ray)) {
      // 葉ノードの場合
      if (!node->child[0] && !node->child[1]) {
        // ノードに含まれる全てのPrimitiveと交差計算
        const int primEnd = node->primitivesOffset + node->nPrimitives;
        for (int i = node->primitivesOffset; i < primEnd; ++i) {
          const int primIdx = primIndices[i];
          IntersectInfo tempInfo;
          if (primitives[primIdx].intersect(ray, tempInfo)) {
            // intersectしたらrayのtmaxを更新
            hit = true;
            ray.tmax = tempInfo.t;
            info = tempInfo;
          }
        }
      }
    } else {
      // 子ノードとの交差判定
      // rayの方向に応じて最適な順番で交差判定をする
      hit |= intersectNode(node->child[ray.dirInvSign[node->axis]], ray, info);
      hit |=
          intersectNode(node->child[1 - ray.dirInvSign[node->axis]], ray, info);
    }

    return hit;
  }

 public:
  BVH(const Polygon& polygon) {
    // PolygonからTriangleを抜き出して追加していく
    for (int f = 0; f < polygon.nFaces(); ++f) {
      primitives.emplace_back(&polygon, f);
    }
  }

  ~BVH() {
    if (root) {
      deleteBVHNode(root);
    }
  }

  // BVHを構築する
  void buildBVH() {
    // 各Primitiveのバウンディングボックスを事前計算
    for (const auto& prim : primitives) {
      bboxes.push_back(prim.calcAABB());
    }

    // 各Primitiveへのインデックスを表す配列を作成
    primIndices.resize(primitives.size());
    std::iota(primIndices.begin(), primIndices.end(), 0);

    // BVHの構築をルートノードから開始
    root = buildBVHNode(0, primitives.size(), primIndices);
    stats.nNodes = stats.nInternalNodes + stats.nLeafNodes;
  }

  // ノード数を返す
  int nNodes() const { return stats.nNodes; }
  // 中間ノード数を返す
  int nInternalNodes() const { return stats.nInternalNodes; }
  // 葉ノード数を返す
  int nLeafNodes() const { return stats.nLeafNodes; }

  // traverseをする
  bool intersect(Ray& ray, IntersectInfo& info) const {
    return intersectNode(root, ray, info);
  }
};

#endif