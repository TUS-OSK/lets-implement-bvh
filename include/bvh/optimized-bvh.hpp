#ifndef _OPTIMIZED_BVH_H
#define _OPTIMIZED_BVH_H
#include <numeric>
#include <stack>
#include <vector>

#include "core/triangle.hpp"

class OptimizedBVH {
 private:
  std::vector<Triangle> primitives;  // Primitive(三角形)の配列

  // ノードを表す構造体
  // NOTE: 32ByteにAlignmentすることでキャッシュ効率を良くする
  struct alignas(32) BVHNode {
    AABB bbox;  // バウンディングボックス
    union {
      uint32_t primIndicesOffset;  // primIndicesへのオフセット
      uint32_t secondChildOffset;  // 2番目の子へのオフセット
    };
    uint16_t nPrimitives{
        0};  // ノードに含まれるPrimitiveの数(中間ノードの場合は0)
    uint8_t axis{0};  // 分割軸
  };

  // BVHの統計情報を表す構造体
  struct BVHStatistics {
    int nNodes{0};          // ノード総数
    int nInternalNodes{0};  // 中間ノードの数
    int nLeafNodes{0};      // 葉ノードの数
  };

  std::vector<BVHNode> nodes;  // ノード配列(深さ優先順)
  BVHStatistics stats;         // BVHの統計情報

  // 葉ノードを配列に追加する
  void addLeafNode(const AABB& bbox, int primStart, int nPrims) {
    BVHNode node;
    node.bbox = bbox;
    node.primIndicesOffset = primStart;
    node.nPrimitives = nPrims;
    nodes.push_back(node);
    stats.nLeafNodes++;
  }

  // 再帰的にBVHのノードを構築していく
  void buildBVHNode(int primStart, int primEnd) {
    // AABBの計算
    AABB bbox;
    for (int i = primStart; i < primEnd; ++i) {
      bbox = mergeAABB(bbox, primitives[i].calcAABB());
    }

    // 含まれるPrimitiveが少ない場合は葉ノードにする
    const int nPrims = primEnd - primStart;
    if (nPrims <= 4) {
      addLeafNode(bbox, primStart, nPrims);
      return;
    }

    // 分割用に各Primitiveの中心点を含むAABBを計算
    // NOTE: bboxをそのまま使ってしまうとsplitが失敗することが多い
    AABB splitAABB;
    for (int i = primStart; i < primEnd; ++i) {
      splitAABB = mergeAABB(splitAABB, primitives[i].calcAABB().center());
    }

    // 分割軸
    const int splitAxis = splitAABB.longestAxis();

    // AABBの分割(等数分割)
    const int splitIdx = primStart + nPrims / 2;
    std::nth_element(primitives.begin() + primStart,
                     primitives.begin() + splitIdx,
                     primitives.begin() + primEnd,
                     [&](const auto& prim1, const auto& prim2) {
                       return prim1.calcAABB().center()[splitAxis] <
                              prim2.calcAABB().center()[splitAxis];
                     });

    // 分割が失敗した場合は葉ノードを作成
    if (splitIdx == primStart || splitIdx == primEnd) {
      std::cout << "splitting failed" << std::endl;
      std::cout << "nPrimitives: " << nPrims << std::endl;
      std::cout << "splitAxis: " << splitAxis << std::endl;
      std::cout << "primStart: " << primStart << std::endl;
      std::cout << "splitIdx: " << splitIdx << std::endl;
      std::cout << "primEnd: " << primEnd << std::endl;
      std::cout << std::endl;
      addLeafNode(bbox, primStart, nPrims);
      return;
    }

    // ノードを配列に追加する. その際に自分の位置を覚えておく
    const int parentOffset = nodes.size();
    BVHNode node;
    node.bbox = bbox;
    node.primIndicesOffset = primStart;
    node.axis = splitAxis;
    nodes.push_back(node);
    stats.nInternalNodes++;

    // 左の子ノードを配列に追加していく
    buildBVHNode(primStart, splitIdx);

    // 右の子へのオフセットを計算し, 親ノードにセットする
    const int secondChildOffset = nodes.size();
    nodes[parentOffset].secondChildOffset = secondChildOffset;

    // 右の子ノードを配列に追加していく
    buildBVHNode(splitIdx, primEnd);
  }

  // 再帰的にBVHのtraverseを行う
  // NOTE:
  // 再帰なし版も実装してみたがこっちの方が早かった(simple-renderingで0.2秒差)
  bool intersectNode(int nodeIdx, const Ray& ray, const Vec3& dirInv,
                     const int dirInvSign[3], IntersectInfo& info) const {
    bool hit = false;
    const BVHNode& node = nodes[nodeIdx];

    // AABBとの交差判定
    if (node.bbox.intersect(ray, dirInv, dirInvSign)) {
      // 葉ノードの場合
      if (node.nPrimitives > 0) {
        // ノードに含まれる全てのPrimitiveと交差計算
        const int primEnd = node.primIndicesOffset + node.nPrimitives;
        for (int i = node.primIndicesOffset; i < primEnd; ++i) {
          if (primitives[i].intersect(ray, info)) {
            // intersectしたらrayのtmaxを更新
            hit = true;
            ray.tmax = info.t;
          }
        }
      }
      // 中間ノードの場合
      else {
        // 子ノードとの交差判定
        // rayの方向に応じて最適な順番で交差判定をする
        if (dirInvSign[node.axis] == 0) {
          hit |= intersectNode(nodeIdx + 1, ray, dirInv, dirInvSign, info);
          hit |= intersectNode(node.secondChildOffset, ray, dirInv, dirInvSign,
                               info);
        } else {
          hit |= intersectNode(node.secondChildOffset, ray, dirInv, dirInvSign,
                               info);
          hit |= intersectNode(nodeIdx + 1, ray, dirInv, dirInvSign, info);
        }
      }
    }

    return hit;
  }

 public:
  OptimizedBVH(const Polygon& polygon) {
    // PolygonからTriangleを抜き出して追加していく
    for (unsigned int f = 0; f < polygon.nFaces(); ++f) {
      primitives.emplace_back(&polygon, f);
    }
  }

  // BVHを構築する
  void buildBVH() {
    // 各Primitiveのバウンディングボックスを事前計算
    std::vector<AABB> bboxes;
    for (const auto& prim : primitives) {
      bboxes.push_back(prim.calcAABB());
    }

    // BVHの構築をルートノードから開始
    buildBVHNode(0, primitives.size());

    // 総ノード数を計算
    stats.nNodes = stats.nInternalNodes + stats.nLeafNodes;
  }

  // ノード数を返す
  int nNodes() const { return stats.nNodes; }
  // 中間ノード数を返す
  int nInternalNodes() const { return stats.nInternalNodes; }
  // 葉ノード数を返す
  int nLeafNodes() const { return stats.nLeafNodes; }

  // 全体のバウンディングボックスを返す
  AABB rootAABB() const {
    if (nodes.size() > 0) {
      return nodes[0].bbox;
    } else {
      return AABB();
    }
  }

  // traverseをする
  bool intersect(const Ray& ray, IntersectInfo& info) const {
    // レイの方向の逆数と符号を事前計算しておく
    const Vec3 dirInv = 1.0f / ray.direction;
    int dirInvSign[3];
    for (int i = 0; i < 3; ++i) {
      dirInvSign[i] = dirInv[i] > 0 ? 0 : 1;
    }
    return intersectNode(0, ray, dirInv, dirInvSign, info);
  }
};

#endif