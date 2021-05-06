#ifndef _BVH_H
#define _BVH_H
#include <numeric>
#include <vector>

#include "triangle.hpp"

class BVH {
 private:
  std::vector<Triangle> primitives;  // Primitive(三角形)の配列
  std::vector<int> primIndices;  // primitivesへのインデックスの配列.
                                 // 分割時にはこれがソートされる

  // ノードを表す構造体
  struct BVHNode {
    AABB bbox;              // バウンディングボックス
    int primIndicesOffset;  // primIndicesへのオフセット
    int nPrimitives;        // ノードに含まれるPrimitiveの数
    int axis;               // 分割軸(traverseの最適化に使う)
    BVHNode* child[2];  // 子ノードへのポインタ, 両方nullptrだったら葉ノード
  };

  // BVHの統計情報を表す構造体
  struct BVHStatistics {
    int nNodes{0};          // ノード総数
    int nInternalNodes{0};  // 中間ノードの数
    int nLeafNodes{0};      // 葉ノードの数
  };

  BVHNode* root;        // ルートノードへのポインタ
  BVHStatistics stats;  // BVHの統計情報

  // 再帰的にBVHのノードを構築していく
  BVHNode* buildBVHNode(int primStart, int primEnd,
                        const std::vector<AABB>& bboxes,
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
    if (nPrims <= 4) {
      // 葉ノードの作成
      node->bbox = bbox;
      node->primIndicesOffset = primStart;
      node->nPrimitives = nPrims;
      node->child[0] = nullptr;
      node->child[1] = nullptr;
      stats.nLeafNodes++;
      return node;
    }

    // 分割用に各Primitiveの中心点を含むAABBを計算
    // NOTE: bboxをそのまま使ってしまうとsplitが失敗することが多い
    AABB splitAABB;
    for (int i = primStart; i < primEnd; ++i) {
      const int primIdx = primIndices[i];
      splitAABB = mergeAABB(splitAABB, bboxes[primIdx].center());
    }

    // 分割軸
    const int splitAxis = splitAABB.longestAxis();

    // 分割点
    const float splitPos = splitAABB.center()[splitAxis];

    // AABBの分割
    const int splitIdx = primStart + nPrims / 2;
    std::nth_element(primIndices.begin() + primStart,
                     primIndices.begin() + splitIdx,
                     primIndices.begin() + primEnd, [&](int idx1, int idx2) {
                       return bboxes[idx1].center()[splitAxis] <
                              bboxes[idx2].center()[splitAxis];
                     });

    // 分割が失敗した場合は葉ノードを作成
    if (splitIdx == primStart || splitIdx == primEnd) {
      std::cout << "splitting failed" << std::endl;
      std::cout << "nPrimitives: " << nPrims << std::endl;
      std::cout << "splitAxis: " << splitAxis << std::endl;
      std::cout << "splitPos: " << splitPos << std::endl;
      std::cout << "primStart: " << primStart << std::endl;
      std::cout << "splitIdx: " << splitIdx << std::endl;
      std::cout << "primEnd: " << primEnd << std::endl;
      std::cout << std::endl;

      node->bbox = bbox;
      node->primIndicesOffset = primStart;
      node->nPrimitives = nPrims;
      node->child[0] = nullptr;
      node->child[1] = nullptr;
      stats.nLeafNodes++;
      return node;
    }

    // 中間ノードに情報をセット
    node->bbox = bbox;
    node->primIndicesOffset = primStart;
    node->nPrimitives = nPrims;
    node->axis = splitAxis;

    // 左の子ノードで同様の計算
    node->child[0] = buildBVHNode(primStart, splitIdx, bboxes, primIndices);
    // 右の子ノードで同様の計算
    node->child[1] = buildBVHNode(splitIdx, primEnd, bboxes, primIndices);
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
  bool intersectNode(const BVHNode* node, const Ray& ray, const Vec3& dirInv,
                     const int dirInvSign[3], IntersectInfo& info) const {
    bool hit = false;

    // AABBとの交差判定
    if (node->bbox.intersect(ray, dirInv, dirInvSign)) {
      if (node->child[0] == nullptr && node->child[1] == nullptr) {
        // 葉ノードの場合
        // ノードに含まれる全てのPrimitiveと交差計算
        const int primEnd = node->primIndicesOffset + node->nPrimitives;
        for (int i = node->primIndicesOffset; i < primEnd; ++i) {
          const int primIdx = primIndices[i];
          if (primitives[primIdx].intersect(ray, info)) {
            // intersectしたらrayのtmaxを更新
            hit = true;
            ray.tmax = info.t;
          }
        }
      } else {
        // 子ノードとの交差判定
        // rayの方向に応じて最適な順番で交差判定をする
        hit |= intersectNode(node->child[dirInvSign[node->axis]], ray, dirInv,
                             dirInvSign, info);
        hit |= intersectNode(node->child[1 - dirInvSign[node->axis]], ray,
                             dirInv, dirInvSign, info);
      }
    }

    return hit;
  }

 public:
  BVH(const Polygon& polygon) {
    // PolygonからTriangleを抜き出して追加していく
    for (unsigned int f = 0; f < polygon.nFaces(); ++f) {
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
    std::vector<AABB> bboxes;
    for (const auto& prim : primitives) {
      bboxes.push_back(prim.calcAABB());
    }

    // 各Primitiveへのインデックスを表す配列を作成
    primIndices.resize(primitives.size());
    std::iota(primIndices.begin(), primIndices.end(), 0);

    // BVHの構築をルートノードから開始
    root = buildBVHNode(0, primitives.size(), bboxes, primIndices);
    stats.nNodes = stats.nInternalNodes + stats.nLeafNodes;
  }

  // ノード数を返す
  int nNodes() const { return stats.nNodes; }
  // 中間ノード数を返す
  int nInternalNodes() const { return stats.nInternalNodes; }
  // 葉ノード数を返す
  int nLeafNodes() const { return stats.nLeafNodes; }
  // 全体のバウンディングボックスを返す
  AABB rootAABB() const { return root->bbox; }

  // traverseをする
  bool intersect(const Ray& ray, IntersectInfo& info) const {
    // レイの方向の逆数と符号を事前計算しておく
    const Vec3 dirInv = 1.0f / ray.direction;
    int dirInvSign[3];
    for (int i = 0; i < 3; ++i) {
      dirInvSign[i] = dirInv[i] > 0 ? 0 : 1;
    }
    return intersectNode(root, ray, dirInv, dirInvSign, info);
  }
};

#endif