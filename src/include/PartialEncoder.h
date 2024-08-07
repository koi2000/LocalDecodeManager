#ifndef PARTIAL_ENCODER_H
#define PARTIAL_ENCODER_H

#include "LocalSplitter.h"
#include "core.h"
#include <string>

/**
 * 修改encoder，直接分割压缩 使用vertex removal，找个办法记下来
 * 同时根据点记录三group交界处
 *
 * 边界在decode的时候需要对齐，自己maintain三个点
 */

class PartialEncoder {
  public:
    PartialEncoder(std::string path);

    void encode(int groupId, int round);

    void encode(int round);

    void encodeInsideOp(int groupId);

    void encodeBoundaryOp(int groupId);

  private:
    void resetState(int groupId);

    bool isRemovable(int groupId, MCGAL::Vertex* v);

    MCGAL::Halfedge* vertexCut(MCGAL::Mesh& mesh, std::set<int>& boundaryIds, std::queue<int>& gateQueue, MCGAL::Halfedge* startH);

    bool boundaryRemovableInVertexRemoval(int inner, int outer, MCGAL::Halfedge* hit);

    void dumpSubMeshNeighbour(int groupId, int round);

    MCGAL::Halfedge* next_boundary(int ogroupId, MCGAL::Halfedge* boundary);

    // void dumpToBuffer();

    // void writeBaseMesh();

    // void dumpBoundaryToBuffer();

    // void dumpFacetSymbolToBuffer();

    // void dumpHalfedgeSymbolToBuffer();

    // void dumpBoundryMergeMessageToBuffer(int prevDataOffset);

    // void encodeFacetSymbolOp(int groupId);

    // void encodeHalfedgeSymbolOp(int groupId);

    // void encodeLocalBoundary(int groupId, std::vector<int>& boundarys);

    bool isBoundaryRemovable(MCGAL::Halfedge* h);
    // // 为网格设置边界
    // void markBoundry();

    // void mergeBoundary();

    // void resetBfsState();

    bool checkCompetition(MCGAL::Vertex* v) const;

    bool arePointsCoplanar(std::vector<MCGAL::Point>& points);

    bool isPlanar(const std::vector<MCGAL::Vertex*>& polygon, float epsilon) const;

    bool willViolateManifold(const std::vector<MCGAL::Halfedge*>& polygon) const;

    bool willViolateManifoldInDup(int inner, int outer, const std::vector<MCGAL::Halfedge*>& inner_oneRing, const std::vector<MCGAL::Halfedge*>& outer_oneRing) ;

  private:
    Graph graph;
    MCGAL::Mesh mesh;
    LocalSplitter splitter;
    std::vector<MCGAL::Mesh> subMeshes;
    std::vector<MCGAL::Halfedge*> seeds;
    // 每个group的压缩轮数
    std::vector<int> compressRounds;
    std::unordered_map<int, int> dup2origin;
    std::unordered_map<int, std::unordered_map<int, int>> origin2dup;
};

#endif